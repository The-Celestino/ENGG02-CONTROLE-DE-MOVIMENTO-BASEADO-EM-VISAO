#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
import os
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def main():
    """
    Este script gera um número definido de blocos com cores e posições
    aleatórias sobre a mesa no ambiente Gazebo, usando as coordenadas
    exatas da mesa do projeto.
    """
    rospy.init_node('block_spawner')
    rospy.loginfo("Aguardando serviço /gazebo/spawn_sdf_model...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Serviço do Gazebo conectado.")

    # --- Parâmetros de Configuração ---

    # 1. Encontrar o caminho para o seu pacote
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('UR5_seletor_caixa')
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Pacote 'UR5_seletor_caixa' não encontrado.")
        return
        
    models_path = os.path.join(package_path, 'models')

    # 2. Definir os modelos disponíveis baseados na estrutura que criamos
    block_models = {
        'red': os.path.join(models_path, 'block_red', 'model.sdf'),
        'green': os.path.join(models_path, 'block_green', 'model.sdf'),
        'blue': os.path.join(models_path, 'block_blue', 'model.sdf')
    }

    # ========================================================================= #
    # 3. DEFINIÇÕES DA MESA - ATUALIZADO COM AS COORDENADAS EXATAS
    # Pose central da mesa 'mesa_escritorio_clone'
    table_pose = {'x': 0.039142, 'y': 0.675668}
    
    # Assumindo um tamanho de mesa de escritório de 1.2m x 0.7m
    table_size = {'x': 0.8, 'y': 0.5} 
    
    # Altura da superfície da mesa (ASSUMIDA) + metade da altura do bloco
    table_surface_z = 0.78  # <-- Se os blocos flutuarem ou afundarem, AJUSTE ESTE VALOR
    block_height_half = 0.03
    z_pos = table_surface_z + block_height_half

    # Reduzimos um pouco a área para os blocos não cairem da borda
    x_range = [table_pose['x'] - table_size['x']/2.5, table_pose['x'] + table_size['x']/2.5]
    y_range = [table_pose['y'] - table_size['y']/2.5, table_pose['y'] + table_size['y']/2.5]
    # ========================================================================= #

    # 4. Quantos blocos gerar
    num_blocks_to_spawn = 5

    # --- Lógica de Geração ---

    spawned_positions = []
    min_distance_between_blocks = 0.2 # Distância mínima para evitar sobreposição

    for i in range(num_blocks_to_spawn):
        # Escolhe uma cor aleatória
        chosen_color = random.choice(list(block_models.keys()))
        model_path = block_models[chosen_color]
        
        # O nome do modelo no Gazebo deve ser único
        model_name = "block_{}_{}".format(chosen_color, i)

        try:
            with open(model_path, "r") as f:
                model_xml = f.read()
        except IOError as e:
            rospy.logerr(f"Erro ao ler o arquivo do modelo {model_path}: {e}")
            continue

        # Encontra uma posição válida que não esteja muito perto de outros blocos
        valid_position_found = False
        for _ in range(100): # Tenta 100 vezes encontrar um local
            x = random.uniform(x_range[0], x_range[1])
            y = random.uniform(y_range[0], y_range[1])
            
            too_close = False
            for pos in spawned_positions:
                dist = ((x - pos['x'])**2 + (y - pos['y'])**2)**0.5
                if dist < min_distance_between_blocks:
                    too_close = True
                    break
            
            if not too_close:
                valid_position_found = True
                break
        
        if not valid_position_found:
            rospy.logwarn("Não foi possível encontrar uma posição válida para o bloco. Pulando.")
            continue

        spawned_positions.append({'x': x, 'y': y})

        # Define a pose do modelo
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z_pos

        rospy.loginfo(f"Gerando '{model_name}' em x={x:.2f}, y={y:.2f}, z={z_pos:.2f}")
        try:
            spawn_model_proxy(model_name, model_xml, "", initial_pose, "world")
        except rospy.ServiceException as e:
            rospy.logerr(f"Falha ao chamar o serviço de spawn: {e}")

        rospy.sleep(0.5)

    rospy.loginfo("Geração de blocos concluída.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass