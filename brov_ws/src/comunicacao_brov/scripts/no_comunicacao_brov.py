#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from BROV_Comunicacao import CalcularVelocidadeSomAgua, TipoEquacao
from tqdm import tqdm
import time

class NoComunicacaoBrov():
    NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA = 'calcular_velocidade_som_agua'

    def __init__(self):
        rospy.init_node(self.NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA, anonymous=False)
    
    def gerar_delay_tramissao(self, coordenadada_origem, coordenadada_destino):
        ##*********************Configuração da Transmissao*********************
        TIPO_EQUACAO =  TipoEquacao.MACKENZIE
        TEMPERATURA  = 17
        SALINIDADE   = 25
        PROFUNDIDADE = 10
        PRESSAO      = None
        LATITUDE     = None
        FREQUENCIA_TEMPO_ESPERA= 0.01
        ##*********************Configuração da Transmissao*********************
        calcula_velocidade_som = CalcularVelocidadeSomAgua(tipoequacao= TIPO_EQUACAO,
                                                           temperatura= TEMPERATURA,
                                                           salinidade= SALINIDADE, 
                                                           profundidade= PROFUNDIDADE,
                                                           pressao= PRESSAO,latitude= LATITUDE )
        #
        if  type(coordenadada_origem).__name__ == 'Twist':
            coordenadada_origem  = (coordenadada_origem.linear.x, 
                                    coordenadada_origem.linear.y,
                                    coordenadada_origem.linear.z)
            coordenadada_destino = (coordenadada_destino.linear.x, 
                                    coordenadada_destino.linear.y,
                                    coordenadada_destino.linear.z)
        #
        delay_comunicacao =  calcula_velocidade_som.calcular_delay_transmissao(coordenadada_origem, coordenadada_destino)
        rospy.loginfo(str.format('Executando Delay: {0}', delay_comunicacao))
        for i in tqdm(range(int (delay_comunicacao)), desc=str.format('Executando Delay (Tempo:. {0})', delay_comunicacao)):
            time.sleep(FREQUENCIA_TEMPO_ESPERA)
        #
        # https://roboticsbackend.com/ros-rate-roscpy-roscpp/
        # rate =  rospy.Rate(delay)
        #
        # while not rospy.is_shutdown():
        #     rospy.loginfo(str.format('Delay: {0}', delay))
        # rate.sleep()



def main(argv):
    no_comu = NoComunicacaoBrov()
    coordenadada_origem_teste  = (2,3,5)
    coordenadada_destino_teste = (3,5,7)
    # coordenadada_origem_teste  = Twist()
    # coordenadada_origem_teste.linear.x = 2
    # coordenadada_origem_teste.linear.y = 3
    # coordenadada_origem_teste.linear.z = 5
    # coordenadada_destino_teste = Twist()   
    # coordenadada_destino_teste.linear.x = 3
    # coordenadada_destino_teste.linear.y = 5
    # coordenadada_destino_teste.linear.z = 7
    no_comu.gerar_delay_tramissao(coordenadada_origem_teste, coordenadada_destino_teste)

if __name__ == "__main__":
    import sys
    main(sys.argv)
    