#!/usr/bin/env python3

# Importando todas as bibliotecas necessárias para o funcionamento do robô
import sys
import copy
import rospy
import numpy as np
import math as m
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import DisplayTrajectory
import cv2 as cv
from pynput import keyboard
from camera import image_read
import time as t

# Variáveis globais que vamos usar durante todo o programa
global current_pose, target_pose_correct_orien
global comp_orien
comp_orien = 1

def pos_take_images():
    """Função que move o braço do robô para a posição ideal para tirar fotos das peças"""
    global target_pose_correct_orien
    print("MOVENDO PARA A POSIÇÃO DA CÂMERA...")
    # Estes são os ângulos das juntas para posicionar a câmera corretamente
    images_goals = [1.3909180384024973, -1.2970095837866964, 1.4559604820238743, -1.75, -1.573950195686849, -0.1819244873697139]
    move_group_interface_arm.go(images_goals, wait=True)
    move_group_interface_arm.stop()
    # Salvamos a orientação correta para usar depois
    target_pose_correct_orien = geometry_msgs.msg.Pose()
    target_pose_correct_orien = move_group_interface_arm.get_current_pose()

def home_pos():
    """Move o braço robótico para a posição inicial de segurança"""
    print("MOVENDO PARA A POSIÇÃO INICIAL...")
    # Posição neutra onde o robô fica "em repouso"
    joint_goals = [0.0, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
    move_group_interface_arm.go(joint_goals, wait=True)
    move_group_interface_arm.stop()

def home_pos_inv():
    """Versão invertida da posição inicial - útil para alcançar certas áreas"""
    print("MOVENDO PARA A POSIÇÃO INICIAL...")
    # Similar à posição inicial, mas com o primeiro ângulo invertido
    joint_goals = [-3.14, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
    move_group_interface_arm.go(joint_goals, wait=True)
    move_group_interface_arm.stop()

def close_gripper():
    """Fecha a garra do robô para segurar objetos"""
    print("FECHANDO A GARRA...")
    move_group_interface_gripper.go(gripper_close, wait=True)
    move_group_interface_gripper.stop()

def open_gripper():
    """Abre a garra do robô para soltar objetos"""
    print("ABRINDO A GARRA...")
    move_group_interface_gripper.go(gripper_open, wait=True)
    move_group_interface_gripper.stop()

def approximation(x, y):
    """
    Posiciona a garra do robô aproximadamente acima da peça que queremos pegar
    x, y: coordenadas da peça no mundo real
    """
    global target_pose, comp_orien, target_pose_correct_orien
    # Define onde queremos que a garra vá
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = 1.1  # Altura segura acima da mesa
    
    # Após algumas tentativas, usa a orientação correta da câmera
    if comp_orien > 2:
        target_pose.orientation = target_pose_correct_orien.pose.orientation
    
    # Executa o movimento
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()
    comp_orien = comp_orien + 1

def take_piece():
    """Desce a garra para efetivamente pegar a peça"""
    global target_pose, comp_orien
    # Desce para a altura da mesa para tocar a peça
    target_pose.position.z = 0.96
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

def convert_m2w(p_px):
    """
    Converte coordenadas da imagem (pixels) para coordenadas do mundo real (metros)
    Esta é a mágica que permite ao robô saber onde estão as peças!
    """
    # Parâmetros da câmera - estes valores foram calibrados previamente
    Cx = 160           # Centro da imagem em X
    Cy = 120.5         # Centro da imagem em Y
    Zc = 0.534395      # Distância da câmera até a mesa
    fx = 199.8938206925  # Fator de escala em X
    fy = 199.8938206925  # Fator de escala em Y

    # Fórmulas de conversão da visão computacional
    x_c = ((p_px[0]-Cx)/fx)*Zc
    y_c = ((p_px[1]-Cy)/fy)*Zc
    
    # Ajustes finais para o sistema de coordenadas do robô
    x_w = x_c + 0.004627
    y_w = 0.758958 - y_c

    return x_w, y_w

def red_pieces(c):
    """Sequência completa para pegar uma peça vermelha e colocá-la no local correto"""
    global current_pose, target_pose
    global incr_r
    
    # Prepara as coordenadas da peça vermelha
    coord_red = []
    coord_red.append(c[0])
    coord_red.append(c[1])
    x_r, y_r = convert_m2w(coord_red)

    # Aproxima da peça vermelha
    approximation(x_r, y_r)
    
    # Abre a garra antes de tentar pegar
    open_gripper()
    
    # Desce e pega a peça
    take_piece()
    
    # Fecha a garra para segurar a peça
    close_gripper()

    # Volta para a posição segura acima da peça
    approximation(x_r, y_r)

    # Vai para casa para se reposicionar
    home_pos()

    # Move para a área onde ficam as peças vermelhas
    approx_goals = [-0.9023952823594588, -0.7322973147618699, 1.2709732199546568, -2.115323358711743, -1.5688081113077317, -0.9020939621886495]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Posiciona exatamente onde vai soltar a peça vermelha
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    # O incr_r faz com que cada peça seja colocada um pouquinho mais longe da anterior
    target_pose.position.x = 0.549634458556 - incr_r
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    
    # Desce até a mesa para soltar
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    # Solta a peça
    open_gripper()

    # Volta para cima para não arrastar nada na mesa
    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    # Retorna para a posição da câmera para procurar mais peças
    pos_take_images()

def blue_pieces(c):
    """Mesma lógica da função anterior, mas para peças azuis"""
    global current_pose, target_pose
    global incr_b

    # Prepara coordenadas da peça azul
    coord_blue = []
    coord_blue.append(c[0])
    coord_blue.append(c[1])

    x_b, y_b = convert_m2w(coord_blue)

    # Aproxima da peça azul
    approximation(x_b, y_b)

    # Pega a peça azul
    take_piece()
    close_gripper()
    
    # Volta para posição segura
    approximation(x_b, y_b)
    home_pos()

    # Move para a área das peças azuis
    approx_goals = [-1.5860453154773948, -0.9473863999013306, 1.6479834864306415, -2.2721118993163927, -1.5642330045305393, -1.5840395145339077]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Solta a peça azul
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    target_pose.position.x = 0.0998218605971 - incr_b
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    open_gripper()

    # Volta para cima e retorna à posição da câmera
    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    pos_take_images()

def green_pieces(c):
    """Mesma lógica das anteriores, mas para peças verdes"""
    global current_pose, target_pose
    global incr_g

    # Prepara coordenadas da peça verde
    coord_green = []
    coord_green.append(c[0])
    coord_green.append(c[1])

    x_g, y_g = convert_m2w(coord_green)

    # Aproxima da peça verde
    approximation(x_g, y_g)

    # Pega a peça verde
    take_piece()
    close_gripper()
    
    # Volta para posição segura
    approximation(x_g, y_g)

    # Para as verdes, usa a posição inicial invertida
    home_pos_inv()

    # Move para a área das peças verdes
    approx_goals = [-2.3516030599181112, -1.0095631474481657, 1.7579949487353765, -2.31427905629861, -1.56649968952454, -2.3444216722389184]
    move_group_interface_arm.go(approx_goals, wait=True)
    move_group_interface_arm.stop()

    # Solta a peça verde
    current_pose_2 = move_group_interface_arm.get_current_pose()
    target_pose.position = current_pose_2.pose.position
    target_pose.orientation = current_pose_2.pose.orientation
    target_pose.position.x = -0.350061671499 - incr_g
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    target_pose.position.z = 0.967
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    open_gripper()

    # Volta para cima e retorna à posição da câmera
    target_pose.position.z = 1.1
    move_group_interface_arm.set_pose_target(target_pose)
    move_group_interface_arm.go(wait=True)
    move_group_interface_arm.stop()

    pos_take_images()
    

# Configurações da garra - valores que fazem ela abrir e fechar
gripper_open = [0.005]    # Garra aberta
gripper_close = [0.24]    # Garra fechada

if __name__=='__main__':
    # Aqui começa o programa principal!
    global target_pose, incr_r, incr_b, incr_g
    
    # Cria o objeto para ler as imagens da câmera
    obj_img = image_read()
    
    # Contadores para espaçar as peças quando as colocarmos nos locais finais
    incr_r = 0  # Incremento para peças vermelhas
    incr_b = 0  # Incremento para peças azuis
    incr_g = 0  # Incremento para peças verdes

    # Inicializa o sistema de controle do robô
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    # Cria o comando principal do robô
    robot = moveit_commander.robot.RobotCommander()

    # Cria os controladores para o braço e para a garra
    group_name_1 = "ur5_arm"
    group_name_2 = "gripper"
    move_group_interface_arm = moveit_commander.move_group.MoveGroupCommander(group_name_1)
    move_group_interface_gripper = moveit_commander.move_group.MoveGroupCommander(group_name_2)

    print("-----------------------------------")
    print("Iniciando o programa de pick and place")
    print("-----------------------------------")   

    # Começa indo para a posição inicial de segurança
    home_pos()

    # Depois vai para a posição onde a câmera consegue ver bem as peças
    pos_take_images()

    # Salva a posição atual
    current_pose = geometry_msgs.msg.Pose()
    current_pose = move_group_interface_arm.get_current_pose()

    # Espera um pouquinho para tudo se estabilizar
    t.sleep(2)
    coord = []

    # Faz a primeira leitura para ver quantas peças de cada cor existem e onde estão
    n_b, n_g, n_r, coord = obj_img.rev_coord()
    print("************************")
    print(("O número de peças de cada cor é - vermelho: {}, azul: {}, verde: {}".format(n_r, n_b, n_g)))
    print(("As coordenadas dos objetos são: {}".format(coord)))
    print("************************")

    # Prepara o objeto que vai guardar onde queremos que o robô vá
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation = current_pose.pose.orientation
    
    # Conta o total de peças para saber quando parar
    nt = n_r + n_b + n_g
    
    # Loop principal - continua até pegar todas as peças
    while(nt > 0):
        # Prioridade: primeiro pega as vermelhas, depois azuis, depois verdes
        if(n_r > 0):
            red_pieces(coord[2])
            incr_r = incr_r + 0.1  # Próxima peça vermelha vai um pouquinho mais longe
        elif(n_b > 0):
            blue_pieces(coord[0])
            incr_b = incr_b + 0.1  # Próxima peça azul vai um pouquinho mais longe
        elif(n_g > 0):
            green_pieces(coord[1])
            incr_g = incr_g + 0.1  # Próxima peça verde vai um pouquinho mais longe
        
        # Espera um pouco antes de verificar novamente
        t.sleep(2)
        
        # Verifica se ainda há peças na mesa
        n_b, n_g, n_r, coord = obj_img.rev_coord()
        print("************************")
        print(("O número de peças de cada cor é - vermelho: {}, azul: {}, verde: {}".format(n_r, n_b, n_g)))
        print(("As coordenadas dos objetos são: {}".format(coord)))
        print("************************")
        
        # Atualiza o contador total
        nt = n_r + n_b + n_g

    # Fecha todas as janelas da câmera quando terminar
    cv.destroyAllWindows()