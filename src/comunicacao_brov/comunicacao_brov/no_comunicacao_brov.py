import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time
from tqdm import tqdm

from .brov_comunicacao import TipoEquacao, CalcularVelocidadeSomAgua

class NoComunicacaoBrov(Node):
    NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA = 'calcular_velocidade_som_agua'
    NOME_TOPIC_GERA_DELAY ='/brov/gera_delay'

    def __init__(self):
        super().__init__(node_name= self.NOME_NO_CALCULA_VELOCIDADE_SOM_AGUA)
        self.declare_parameter('coordenadada_origem')
        self.declare_parameter('coordenadada_destino')
        self.gerar_delay_tramissao()
        # self.pub_no = self.create_publisher(Twist,self.NOME_TOPIC_GERA_DELAY, 10 )

    def validar_valores_parametros(self, coordenadada_origem:Parameter, coordenadada_destino:Parameter ):
        # print(f'coordenadada_origem Parametro {coordenadada_origem.type_}')
        # print(f'coordenadada_destino Parametro {coordenadada_destino.type_}')
        return True        
    
    def gerar_delay_tramissao(self):
        ##*********************Configuração da Transmissao*********************
        TIPO_EQUACAO =  TipoEquacao.MACKENZIE
        TEMPERATURA  = 17
        SALINIDADE   = 25
        PROFUNDIDADE = 10
        PRESSAO      = None
        LATITUDE     = None
        FREQUENCIA_TEMPO_ESPERA= 0.01
        ##*********************Configuração da Transmissao*********************
        coordenadada_origem=  self.get_parameter_or('coordenadada_origem', None)
        coordenadada_destino= self.get_parameter_or('coordenadada_destino', None)
        if self.validar_valores_parametros(coordenadada_origem, coordenadada_destino):
            #
            calcula_velocidade_som = CalcularVelocidadeSomAgua(tipoequacao= TIPO_EQUACAO,
                                                               temperatura= TEMPERATURA,
                                                               salinidade= SALINIDADE, 
                                                               profundidade= PROFUNDIDADE,
                                                               pressao= PRESSAO,latitude= LATITUDE)
            #
            coordenadada_origem = (coordenadada_origem.value[0],coordenadada_origem.value[1],coordenadada_origem.value[2])
            coordenadada_destino = (coordenadada_destino.value[0],coordenadada_destino.value[1],coordenadada_destino.value[2])
            #
            delay_comunicacao =  calcula_velocidade_som.calcular_delay_transmissao(coordenadada_origem, coordenadada_destino)
            print(str.format('Temperatura: {0}', TEMPERATURA))
            print(str.format('Salinidade: {0}', SALINIDADE))
            print(str.format('Profundidade: {0}', SALINIDADE))
            print(str.format('Equacao Executada: {0}', str(TIPO_EQUACAO)))
            print(str.format('Calculo de Delay: {0}', delay_comunicacao))
            #
            for i in tqdm(range(int (delay_comunicacao)), desc=str.format('Executando Delay (Tempo:. {0})', delay_comunicacao)):
                time.sleep(FREQUENCIA_TEMPO_ESPERA)

def main(args=None):
    rclpy.init(args=args)
    no_comunicacao = NoComunicacaoBrov()
    # rclpy.spin(no_comunicacao)
    no_comunicacao.destroy_node()
    rclpy.shutdown()
    # **************Para fazer testes no terminal*****************
    # ros2 run comunicacao_brov no_comunicacao_brov --ros-args -p coordenadada_origem:=[14,50,36] -p coordenadada_destino:="[4.4, 5.5, 6.6]"

if __name__ == "__main__":
    try:
        main()       
    except :
        pass