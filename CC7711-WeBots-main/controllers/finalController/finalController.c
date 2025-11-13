#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/lidar.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PASSO_TEMPO 32

#define VEL_AVANCO 3.0
#define GANHO_CURVA 2.0
#define VEL_ROTACAO 2.0
#define VEL_RE -2.0

#define LIMIAR_OBSTACULO 80.0
#define LIMIAR_CRITICO 150.0
#define LIMIAR_LATERAL 60.0

#define DISTANCIA_DETECCAO 0.1

#define MAX_OBJETOS 64
#define PREFIXO_DEF_OBJETO "CAIXA"

typedef struct {
	WbNodeRef no_ref;
	char nome_def[32];
	double massa_valor;
	int tem_massa_definida;
} DadosObjeto;

static double obter_massa_solido(WbNodeRef no_solido, int *tem_massa_explicita) {
	if (!no_solido)
		return INFINITY;

	*tem_massa_explicita = 0;

	WbFieldRef campo_massa = wb_supervisor_node_get_field(no_solido, "mass");
	if (campo_massa) {
		double massa = wb_supervisor_field_get_sf_float(campo_massa);
		if (massa > 0.0) {
			*tem_massa_explicita = 1;
			return massa;
		}
	}

	WbFieldRef campo_tamanho = wb_supervisor_node_get_field(no_solido, "size");
	if (campo_tamanho) {
		const double *tamanho = wb_supervisor_field_get_sf_vec3f(campo_tamanho);
		double volume = tamanho[0] * tamanho[1] * tamanho[2];
		return volume;
	}

	return INFINITY;
}

static WbNodeRef encontrar_alvo_mais_leve(char *def_saida, size_t tamanho_def_saida) {
	DadosObjeto lista_objetos[MAX_OBJETOS];
	int total_objetos = 0;
	int flag_massa_explicita = 0;

	printf("--- INICIANDO BUSCA PELO ALVO MAIS LEVE ---\n");

	for (int i = 1; i <= MAX_OBJETOS; ++i) {
		char nome_atual[32];
		snprintf(nome_atual, sizeof(nome_atual), PREFIXO_DEF_OBJETO "%02d", i);
		WbNodeRef no_atual = wb_supervisor_node_get_from_def(nome_atual);
		if (!no_atual)
			continue;

		double massa = obter_massa_solido(no_atual, &flag_massa_explicita);

		if (massa > 0.0 && massa != INFINITY) {
			strcpy(lista_objetos[total_objetos].nome_def, nome_atual);
			lista_objetos[total_objetos].no_ref = no_atual;
			lista_objetos[total_objetos].massa_valor = massa;
			lista_objetos[total_objetos].tem_massa_definida = flag_massa_explicita;
			total_objetos++;

			printf("Objeto encontrado %s: %s = %.3f\n", nome_atual,
				   flag_massa_explicita ? "MASSA" : "volume", massa);
		}
	}

	if (total_objetos == 0) {
		printf("[AVISO] Nenhum objeto com massa/volume válido foi encontrado.\n");
		return NULL;
	}

	printf("\n--- ANALISANDO %d OBJETOS VÁLIDOS ---\n", total_objetos);

	DadosObjeto melhor_por_massa = {0};
	int achou_por_massa = 0;

	for (int i = 0; i < total_objetos; i++) {
		if (lista_objetos[i].tem_massa_definida) {
			if (!achou_por_massa || lista_objetos[i].massa_valor < melhor_por_massa.massa_valor) {
				melhor_por_massa = lista_objetos[i];
				achou_por_massa = 1;
				printf("-> Nova melhor massa explícita: %s (%.3f)\n", lista_objetos[i].nome_def, lista_objetos[i].massa_valor);
			}
		}
	}

	DadosObjeto melhor_por_volume = {0};
	int achou_por_volume = 0;

	for (int i = 0; i < total_objetos; i++) {
		if (!achou_por_volume || lista_objetos[i].massa_valor < melhor_por_volume.massa_valor) {
			melhor_por_volume = lista_objetos[i];
			achou_por_volume = 1;
		}
	}

	DadosObjeto alvo_final;
	if (achou_por_massa) {
		alvo_final = melhor_por_massa;
		printf("\n>> SELEÇÃO FINAL POR MASSA EXPLÍCITA\n");
	} else if (achou_por_volume) {
		alvo_final = melhor_por_volume;
		printf("\n>> SELEÇÃO FINAL POR VOLUME (FALLBACK)\n");
	} else {
		printf("[ERRO] Falha ao selecionar um objeto alvo.\n");
		return NULL;
	}

	printf(">> ALVO DEFINIDO: %s (Valor: %.3f)\n", alvo_final.nome_def, alvo_final.massa_valor);

	if (def_saida && tamanho_def_saida) {
		strncpy(def_saida, alvo_final.nome_def, tamanho_def_saida);
		def_saida[tamanho_def_saida - 1] = '\0';
	}

	return alvo_final.no_ref;
}

static double calcular_angulo_para_alvo(double pos_robo_x, double pos_robo_y, double angulo_robo,
										double pos_alvo_x, double pos_alvo_y) {
	double delta_x = pos_alvo_x - pos_robo_x;
	double delta_y = pos_alvo_y - pos_robo_y;
	double angulo_para_alvo = atan2(delta_y, delta_x);

	double diff_angulo = angulo_para_alvo - angulo_robo;
	while (diff_angulo > M_PI)
		diff_angulo -= 2 * M_PI;
	while (diff_angulo < -M_PI)
		diff_angulo += 2 * M_PI;

	return diff_angulo;
}

static double obter_orientacao_robo(const double *matriz_rotacao) {
	return atan2(matriz_rotacao[3], matriz_rotacao[0]);
}

typedef enum { NAVEGACAO = 0, EVASAO = 1, ROTACAO_FINAL = 2, DESVIANDO = 3 } ModoOperacao;

int main() {
	wb_robot_init();

	WbDeviceTag motor_esq = wb_robot_get_device("left wheel motor");
	WbDeviceTag motor_dir = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(motor_esq, INFINITY);
	wb_motor_set_position(motor_dir, INFINITY);

	WbNodeRef no_robo = wb_supervisor_node_get_self();
	char def_alvo[32] = {0};
	WbNodeRef no_alvo = encontrar_alvo_mais_leve(def_alvo, sizeof(def_alvo));

	if (!no_alvo) {
		printf("[ERRO FATAL] Nenhum alvo válido encontrado. Encerrando.\n");
		wb_robot_cleanup();
		return 1;
	}

	WbDeviceTag sensores_prox[8] = {0};
	const char *nomes_sensores_prox[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
	double valores_sensores_prox[8] = {0};

	for (int i = 0; i < 8; ++i) {
		sensores_prox[i] = wb_robot_get_device(nomes_sensores_prox[i]);
		if (sensores_prox[i]) {
			wb_distance_sensor_enable(sensores_prox[i], PASSO_TEMPO);
			printf("Sensor %s ativado.\n", nomes_sensores_prox[i]);
		}
	}
	printf("Sensores de proximidade prontos.\n");

	ModoOperacao estado_atual = NAVEGACAO;
	double inicio_evasao = 0.0;
	int dir_evasao = 1;
	double ultimo_log_dist = 0.0;
	int passos_sem_progresso = 0;
	double ultima_distancia = 1e9;
	int contador_desvio = 0;

	const double *pos_robo_inicial = wb_supervisor_node_get_position(no_robo);
	const double *pos_alvo_inicial = wb_supervisor_node_get_position(no_alvo);
	double dist_inicial = sqrt(pow(pos_alvo_inicial[0] - pos_robo_inicial[0], 2) + pow(pos_alvo_inicial[1] - pos_robo_inicial[1], 2));

	printf("\n--- STATUS INICIAL ---\n");
	printf("Posição Robô: (%.3f, %.3f)\n", pos_robo_inicial[0], pos_robo_inicial[1]);
	printf("Posição Alvo %s: (%.3f, %.3f)\n", def_alvo, pos_alvo_inicial[0], pos_alvo_inicial[1]);
	printf("Distância para o alvo: %.3fm\n", dist_inicial);

	while (wb_robot_step(PASSO_TEMPO) != -1) {
		for (int i = 0; i < 8; i++) {
			if (sensores_prox[i]) {
				valores_sensores_prox[i] = wb_distance_sensor_get_value(sensores_prox[i]);
			}
		}

		const double *pos_robo_atual = wb_supervisor_node_get_position(no_robo);
		const double *rot_robo_atual = wb_supervisor_node_get_orientation(no_robo);
		double angulo_robo_atual = obter_orientacao_robo(rot_robo_atual);

		const double *pos_alvo_atual = wb_supervisor_node_get_position(no_alvo);
		double delta_x = pos_alvo_atual[0] - pos_robo_atual[0];
		double delta_y = pos_alvo_atual[1] - pos_robo_atual[1];
		double dist_atual = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

		double angulo_para_alvo = calcular_angulo_para_alvo(pos_robo_atual[0], pos_robo_atual[1],
															angulo_robo_atual, pos_alvo_atual[0], pos_alvo_atual[1]);

		if (fabs(dist_atual - ultimo_log_dist) > 0.1 || estado_atual == ROTACAO_FINAL) {
			char nome_estado_atual[20];
			if (estado_atual == NAVEGACAO)
				strcpy(nome_estado_atual, "NAVEGACAO");
			else if (estado_atual == EVASAO)
				strcpy(nome_estado_atual, "EVASAO");
			else if (estado_atual == DESVIANDO)
				strcpy(nome_estado_atual, "DESVIANDO");
			else
				strcpy(nome_estado_atual, "ROTACAO_FINAL");

			printf("Distância: %.2fm | Alvo: %s | Estado: %s\n",
				   dist_atual, def_alvo, nome_estado_atual);
			ultimo_log_dist = dist_atual;
		}

		int obs_frente = 0;
		int obs_esquerda = 0;
		int obs_direita = 0;
		int obs_critico = 0;

		if (valores_sensores_prox[0] > LIMIAR_OBSTACULO || valores_sensores_prox[7] > LIMIAR_OBSTACULO) {
			obs_frente = 1;
		}

		if (valores_sensores_prox[0] > LIMIAR_CRITICO || valores_sensores_prox[7] > LIMIAR_CRITICO) {
			obs_critico = 1;
		}

		if (valores_sensores_prox[5] > LIMIAR_LATERAL || valores_sensores_prox[6] > LIMIAR_LATERAL) {
			obs_esquerda = 1;
		}

		if (valores_sensores_prox[1] > LIMIAR_LATERAL || valores_sensores_prox[2] > LIMIAR_LATERAL) {
			obs_direita = 1;
		}

		if (obs_frente || obs_esquerda || obs_direita) {
			// printf("DEBUG SENSORES: ");
			// for (int i = 0; i < 8; i++) {
			// 	printf("%s:%.0f ", nomes_sensores_prox[i], valores_sensores_prox[i]);
			// }
			// printf("\n");
		}

		if (estado_atual != ROTACAO_FINAL) {
			if (dist_atual < DISTANCIA_DETECCAO) {
				estado_atual = ROTACAO_FINAL;
				printf("\n*** ALVO '%s' ALCANÇADO A %.2fm! INICIANDO ROTAÇÃO FINAL! ***\n",
					   def_alvo, dist_atual);
			} else if (obs_critico && estado_atual != EVASAO) {
				estado_atual = EVASAO;
				inicio_evasao = wb_robot_get_time();
				dir_evasao = (obs_esquerda || valores_sensores_prox[5] > valores_sensores_prox[2]) ? 1 : -1;
				printf("ALERTA: Obstáculo crítico! Iniciando manobra de evasão (%s)\n",
					   dir_evasao > 0 ? "DIREITA" : "ESQUERDA");
			} else if (obs_frente && estado_atual == NAVEGACAO) {
				estado_atual = DESVIANDO;
				contador_desvio = 0;
				if (!obs_direita && obs_esquerda) {
					dir_evasao = -1;
				} else if (!obs_esquerda && obs_direita) {
					dir_evasao = 1;
				} else {
					dir_evasao = (rand() % 2) ? 1 : -1;
				}
				printf("Atenção: Obstáculo frontal. Iniciando desvio (%s)\n",
					   dir_evasao > 0 ? "ESQUERDA" : "DIREITA");
			} else if (estado_atual == DESVIANDO) {
				contador_desvio++;
				if (contador_desvio > 30 && !obs_frente) {
					estado_atual = NAVEGACAO;
					printf("Info: Desvio concluído. Retomando navegação.\n");
					ultima_distancia = dist_atual;
				}
			} else if (estado_atual == EVASAO) {
				double agora = wb_robot_get_time();
				if (agora - inicio_evasao > 2.0) {
					estado_atual = NAVEGACAO;
					printf("Info: Evasão concluída. Retomando navegação.\n");
					ultima_distancia = dist_atual;
				}
			}
		}

		if (estado_atual == NAVEGACAO) {
			double vel_base = VEL_AVANCO;
			double ganho_ajuste = GANHO_CURVA;

			if (fabs(angulo_para_alvo) > 1.0) {
				vel_base *= 0.5;
			} else if (dist_atual < 0.4) {
				vel_base *= 0.7;
			}

			if (obs_esquerda && !obs_direita) {
				angulo_para_alvo -= 0.3;
			} else if (obs_direita && !obs_esquerda) {
				angulo_para_alvo += 0.3;
			}

			double vel_esq = vel_base - angulo_para_alvo * ganho_ajuste;
			double vel_dir = vel_base + angulo_para_alvo * ganho_ajuste;

			wb_motor_set_velocity(motor_esq, vel_esq);
			wb_motor_set_velocity(motor_dir, vel_dir);

			if (dist_atual < ultima_distancia - 0.02) {
				passos_sem_progresso = 0;
				ultima_distancia = dist_atual;
			} else {
				passos_sem_progresso++;
				if (passos_sem_progresso > 100) {
					printf("Aviso: Sem progresso detectado. Tentando corrigir rota.\n");
					passos_sem_progresso = 0;
				}
			}

		} else if (estado_atual == EVASAO) {
			double agora = wb_robot_get_time();
			double delta_t = agora - inicio_evasao;

			if (delta_t < 0.8) {
				wb_motor_set_velocity(motor_esq, VEL_RE * 1.2);
				wb_motor_set_velocity(motor_dir, VEL_RE * 1.2);
			} else if (delta_t < 1.8) {
				wb_motor_set_velocity(motor_esq, dir_evasao * VEL_AVANCO * 0.8);
				wb_motor_set_velocity(motor_dir, -dir_evasao * VEL_AVANCO * 0.8);
			} else {
				wb_motor_set_velocity(motor_esq, VEL_AVANCO * 0.6);
				wb_motor_set_velocity(motor_dir, VEL_AVANCO * 0.6);
			}

		} else if (estado_atual == DESVIANDO) {
			double forca_curva = 1.5;
			if (obs_frente) {
				forca_curva = 2.5;
			}

			wb_motor_set_velocity(motor_esq, VEL_AVANCO * 0.7 - dir_evasao * forca_curva);
			wb_motor_set_velocity(motor_dir, VEL_AVANCO * 0.7 + dir_evasao * forca_curva);

		} else {
			wb_motor_set_velocity(motor_esq, -VEL_ROTACAO);
			wb_motor_set_velocity(motor_dir, VEL_ROTACAO);
		}
	}

	wb_robot_cleanup();
	return 0;
}