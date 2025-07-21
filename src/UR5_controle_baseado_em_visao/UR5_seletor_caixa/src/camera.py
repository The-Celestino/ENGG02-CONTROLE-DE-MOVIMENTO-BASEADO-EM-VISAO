#!/usr/bin/env python3
"""
Nó Python para leitura de dados da câmera
"""

import sys, time
import numpy as np
import cv2 as cv
import roslib
import rospy
from sensor_msgs.msg import CompressedImage  # Mensagens do ROS

print(('versão do opencv: ', cv.__version__))

class LeitorDeImagem:
    def __init__(self):
        # Define o tópico de inscrição
        self.subscriber = rospy.Subscriber(("/myur5/camera1/image_raw/compressed"), 
                                           CompressedImage, self.callback, queue_size=1)

    def callback(self, ros_data):
        global coordenadas, n_azul, n_verde, n_vermelho
        coordenadas = []

        def desenhar(mascara, cor):
            n = 0
            coord = []
            contornos, hierarquia = cv.findContours(mascara, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            for c in contornos:
                area = cv.contourArea(c)
                if area > 100:
                    n = n + 1
                    m = cv.moments(c)
                    if m["m00"] == 0:
                        m["m00"] = 1
                    x = int(m["m10"] / m["m00"])
                    y = int(m["m01"] / m["m00"])
                    cv.circle(frame, (x, y), 3, cor, -1)
                    fonte = cv.FONT_HERSHEY_SIMPLEX
                    cv.putText(frame, "(" + str(x) + ", " + str(y) + ")", (x + 28, y), fonte, 0.5, cor, 1, cv.LINE_AA)
                    casco_convexo = cv.convexHull(c)
                    cv.drawContours(frame, [casco_convexo], 0, cor, 3)
                    coord.append(x)
                    coord.append(y)
            return n, coord

        """Aqui as imagens são lidas e processadas"""
        # VERMELHO
        vermelhoBaixo1 = np.array([0, 100, 20], np.uint8)
        vermelhoAlto1 = np.array([8, 255, 255], np.uint8)
        vermelhoBaixo2 = np.array([175, 100, 20], np.uint8)
        vermelhoAlto2 = np.array([179, 255, 255], np.uint8)

        # AZUL
        azulBaixo = np.array([100, 100, 20], np.uint8)
        azulAlto = np.array([125, 255, 255], np.uint8)

        # VERDE
        verdeBaixo = np.array([45, 100, 20], np.uint8)
        verdeAlto = np.array([95, 255, 255], np.uint8)

        np_arr = np.frombuffer(ros_data.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mascaraVermelho1 = cv.inRange(hsv, vermelhoBaixo1, vermelhoAlto1)
        mascaraVermelho2 = cv.inRange(hsv, vermelhoBaixo2, vermelhoAlto2)
        mascaraVermelho = cv.bitwise_or(mascaraVermelho1, mascaraVermelho2)
        mascaraAzul = cv.inRange(hsv, azulBaixo, azulAlto)
        mascaraVerde = cv.inRange(hsv, verdeBaixo, verdeAlto)

        pecas_azuis = []
        pecas_verdes = []  # Vetores que armazenam a posição em pixels de cada uma das peças
        pecas_vermelhas = []

        n_azul, pecas_azuis = desenhar(mascaraAzul, (255, 0, 0))
        coordenadas.append(pecas_azuis)
        n_verde, pecas_verdes = desenhar(mascaraVerde, (0, 255, 0))
        coordenadas.append(pecas_verdes)
        n_vermelho, pecas_vermelhas = desenhar(mascaraVermelho, (0, 0, 255))
        coordenadas.append(pecas_vermelhas)

        cv.imshow('frame', frame)
        cv.waitKey(2)
    
    def revisar_coordenadas(self):
        global coordenadas, n_azul, n_verde, n_vermelho
        return n_azul, n_verde, n_vermelho, coordenadas
        
    
def main(args):
    """Inicializa e finaliza o nó ROS"""
    ic = LeitorDeImagem()
    rospy.init_node('leitor_de_imagem', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Encerrando o nó ROS Leitor de Imagem')
        cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
