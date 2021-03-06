#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty
from BROV_Comunicacao import CalcularVelocidadeSomAgua, TipoEquacao
from tqdm import tqdm
import time

class NoComunicacaoBrov():
    NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA = 'calcular_velocidade_som_agua'
    NOME_TOPIC_GERA_DELAY ='/brov/gera_delay'

    def __init__(self):
        self.__inicializa_no()

    def __inicializa_no(self):
        rospy.init_node(self.NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA, anonymous=False)
        self.pub_no = rospy.Publisher(self.NOME_TOPIC_GERA_DELAY, Float32, queue_size=10 )
    
    def validar_valores_parametros(self, argv):
        # print(f'coordenadada_origem Parametro {coordenadada_origem.type_}')
        # print(f'coordenadada_destino Parametro {coordenadada_destino.type_}')
        return True 

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
            # time.sleep(FREQUENCIA_TEMPO_ESPERA)
            rospy.sleep(FREQUENCIA_TEMPO_ESPERA)
        
        self.pub_no.publish(delay_comunicacao)
        #
        # https://roboticsbackend.com/ros-rate-roscpy-roscpp/
        # rate =  rospy.Rate(delay)
        #
        # while not rospy.is_shutdown():
        #     rospy.loginfo(str.format('Delay: {0}', delay))
        #     rate.sleep()



def main(argv):
    no_comu = NoComunicacaoBrov()
    if no_comu.validar_valores_parametros(argv) == False:
        rospy.ROSInterruptException
    #
    coordenadada_origem_teste  = tuple(map(float, argv[1:4]))
    coordenadada_destino_teste = tuple(map(float, argv[4:7]))
    no_comu.gerar_delay_tramissao(coordenadada_origem_teste, coordenadada_destino_teste)
    

if __name__ == "__main__":
    try:
        import sys
        main(sys.argv)        
    except rospy.ROSInterruptException:
        pass