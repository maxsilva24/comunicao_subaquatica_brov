#!/usr/bin/env python2.7
# -*- coding: UTF-8 -*-

###########################################################################
#    Copyright (C) 2007 by Justin Eskesen and Josep Miquel Jornet Montana                                     
#    <jge@mit.edu> <jmjornet@mit.edu>                                                            
#
# Copyright: See COPYING file that comes with this distribution
#
###########################################################################

from enum import Enum
import math
import sys, random
import time
# import AUVNetSim.Simulation as AUVSim
# import pylab

class TipoEquacao(Enum): 
    MACKENZIE = 1
    """
    Equação Mackenzie
    Faixa de validade: temperatura 2 a 30°C, salinidade 25 a 40 partes por mil, profundidade 0 a 8000 m 
    """
    COPPENS = 2
    """
    Equação Coppens
    Faixa de validade: temperatura de 0 a 35 °C, salinidade de 0 a 45 partes por mil, profundidade de 0 a 4000 m
    """
    UNESCO = 3
    """
    Equação Unesco
    Faixa de validade: temperatura de 0 a 40 °C, salinidade de 0 a 40 partes por mil, pressão de 0 a 1000 bar (Wong e Zhu, 1995).
    """
    DEL_GROSSO = 4
    """
    Equação Del Grosso
    Faixa de validade: temperatura de 0 a 30°C, salinidade de 30 a 40 partes por mil, pressão de 0 a 1000 kg/cm², onde 100 kPa=1,019716 kg/cm2. (Wong e Zhu, 1995)    
    """
    NPL = 5
    """
    Equação NPL
    Faixa de validade: Esta equação é válida para uso em qualquer oceano ou mar com uma salinidade que não exceda 42.
    """    

class CalcularVelocidadeSomAgua():       
    def __init__(self, tipoequacao, temperatura, salinidade, profundidade = None, pressao=None, latitude=None ):
        self.tipoequacao  = tipoequacao
        self.temperatura  = temperatura
        self.salinidade   = salinidade 
        self.profundidade = profundidade 
        self.pressao      = pressao
        self.latitude     = latitude

    def calcular_velocidade_som(self, tipoequacao, 
                                temperatura, salinidade, 
                                pressao =None, profundidade=None, latitude=None):
        """
            Calcula a velocidade do som na agua de acordo com o tipo de equação
        Args:
            temperatura (float): temperatura em graus Celsius
            salinidade (float): salinidade em partes por mil
            profundidade (float): profundidade em metros. Defaults to None.
            pressao (float): pressão em quilopascal. Defaults to None.
            latitude(float): latitude do objeto. Defaults to None.
        Returns:
            velocidade_som(float): Velocidade do som na agua 
        """
        if tipoequacao == TipoEquacao.MACKENZIE :
            return self.calcular_equacao_mackenzie(temperatura, salinidade, profundidade)
        elif tipoequacao == TipoEquacao.COPPENS :
            return self.calcular_equacao_coppens(temperatura, salinidade, profundidade)
        elif tipoequacao == TipoEquacao.UNESCO :
            return self.calcular_equacao_unesco(temperatura, salinidade, pressao)
        elif tipoequacao == TipoEquacao.DEL_GROSSO :
            return self.calcular_equacao_del_grosso(temperatura, salinidade, pressao)
        elif tipoequacao == TipoEquacao.NPL :
            # return self.equacao_NPL(temperatura, salinidade, profundidade, latitude)   
            print('Falta em construção')    

    def calcular_equacao_mackenzie(self, temperatura, salinidade, profundidade):
        """
            A equação para a velocidade do som na água do mar em função da temperatura, 
            salinidade e profundidade é dada pela equação de Mackenzie (1981).
            Faixa de validade: temperatura 2 a 30°C, salinidade 25 a 40 partes por mil, profundidade 0 a 8000 m
            link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_mackenzie
        Args:
            temperatura (float): temperatura em graus Celsius (2 a 30°C)
            salinidade (float): salinidade em partes por mil (25 a 40)
            profundidade (float): profundidade em metros (0 a 8000m)
        Returns
            velocidade_som(float): Velocidade do som na agua             
        """
        temperatura = float(temperatura)
        salinidade = float(salinidade)
        profundidade = float(profundidade)
        #
        if not (temperatura >= 2 and temperatura <= 30):
            raise Exception(
                'Informe uma Temperatura válida entre 2 a 30 graus Celsius')
        #
        if not (salinidade >= 25 and salinidade <= 40):
            raise Exception(
                'Informe uma salinidade válida 25 a 40 partes por mil')
        #
        if not (profundidade >= 0 and profundidade <= 8000):
            raise Exception(
                'Informe uma profundidade válida entre 0 a 8000 metros')
        #
        # c(D,S,T) = 1448.96 + 4.591T - 5.304 x 10⁻²T² + 2.374 x 10⁻⁴T³ + 1.340 (S-35) +
        #            + 1.630 x 10⁻²D + 1.675 x 10⁻⁷D² - 1.025 x 10⁻²T(S - 35) - 7.139 x 10⁻¹³TD³
        resultado_equacao = 1448.96 + 4.591 * temperatura \
                            - 5.304 * pow(10,-2) * pow(temperatura,2) \
                            + 2.374 * pow(10,-4) * pow(temperatura,3) \
                            + 1.340 * (salinidade-35) \
                            + 1.630 * pow(10,-2) * profundidade \
                            + 1.675 * pow(10,-7) * pow(profundidade,2) \
                            - 1.025 * pow(10,-2) * temperatura * (salinidade - 35) \
                            - 7.139 * pow(10,-13) * temperatura * pow(profundidade,3)
        #
        return resultado_equacao

    def calcular_equacao_coppens(self, temperatura, salinidade, profundidade):
        """
            A equação para a velocidade do som na água do mar em função da temperatura, 
            salinidade e profundidade é dada pela equação de Coppens (1981).
            Faixa de validade: temperatura de 0 a 35 °C, salinidade de 0 a 45 partes por mil, profundidade de 0 a 4000 m
            link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_coppens
        Args:
            temperatura (float): temperatura em graus Celsius (0 a 35°C)
            salinidade (float): salinidade em partes por mil  (0 a 45)
            profundidade (float): profundidade em metros      (0 a 4000)
        Returns
            velocidade_som(float): Velocidade do som na agua             
        """
        temperatura = float(temperatura)/10  #T/10
        salinidade = float(salinidade)
        profundidade = float(profundidade)/1000  #profundidade deve ser quilômetros
        #
        if not (temperatura >= 0 and temperatura <= 3.5):
            raise Exception(
                'Informe uma Temperatura válida entre 0 a 35 graus Celsius')
        #
        if not (salinidade >= 0 and salinidade <= 45):
            raise Exception(
                'Informe uma salinidade válida 0 a 45 partes por mil')
        #
        if not (profundidade >= 0 and profundidade <= 4):
            raise Exception(
                'Informe uma profundidade válida entre 0 a 4000 metros')
        #
        # c(0,S,t) = 1449.05 + 45.7t - 5.21t² + 0.23t³ + (1.333 - 0.126t + 0.009t²)(S - 35)
        resultado_equacao_1 = 1449.05 \
                              + 45.7 * temperatura \
                              - 5.21 * pow(temperatura,2) \
                              + 0.23 * pow(temperatura,3) \
                              + (1.333 - 0.126 * temperatura + 0.009 * pow(temperatura,2)) * (salinidade - 35)
        # c(D,S,t) = c(0,S,t) + (16.23 + 0.253t)D + (0.213-0.1t)D² + [0.016 + 0.0002(S-35)](S - 35)tD
        resultado_equacao = resultado_equacao_1 \
                            + (16.23 + 0.253 * temperatura) * profundidade \
                            + (0.213-0.1 * temperatura) * pow(profundidade,2) \
                            + (0.016 + 0.0002 * (salinidade-35)) * (salinidade - 35) * temperatura * profundidade
        #
        return resultado_equacao
    
    def calcular_equacao_unesco(self, temperatura, salinidade, pressao):
            """
                O algoritmo padrão internacional, muitas vezes conhecido como algoritmo da UNESCO, é devido a Chen e Millero (1977), 
                usa a pressão como variável em vez de profundidade.
                Faixa de validade: temperatura de 0 a 40 °C, salinidade de 0 a 40 partes por mil, pressão de 0 a 1000 bar (Wong e Zhu, 1995).
                link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_unesco
            Args:
                temperatura (float): temperatura em graus Celsius (0 a 40°C)
                salinidade (float): salinidade em partes por mil  (0 a 40)
                pressao (float): pressão em quilopascal  (0 a 100000 Kpa)
            Returns
                velocidade_som(float): Velocidade do som na agua        
            """
            temperatura = float(temperatura)
            salinidade = float(salinidade)
            pressao = float(pressao) / 100 #em bar
            #
            if not (temperatura >= 0 and temperatura <= 40):
                raise Exception(
                    'Informe uma Temperatura válida entre 0 a 40 graus Celsius')
            #
            if not (salinidade >= 0 and salinidade <= 40):
                raise Exception(
                    'Informe uma salinidade válida 0 a 40 partes por mil')
            #
            if not (pressao >= 0 and pressao <= 1000):
                raise Exception(
                    'Informe uma pressão válida entre 0 a 100000 KPas ou 1000 bares')
            #
            coef_A = [  #A00 1.389      A01  -1.262E-2  A02  7.166E-5    A03  2.008E-6  A04  -3.21E-8
                     [    1.389     ,    -1.262E-2 ,      7.166E-5,        2.008E-6 ,     -3.21E-8 ],
                     #A10 9.4742E-5  A11  -1.2583E-5 A12  -6.4928E-8  A13  1.0515E-8 A14  -2.0142E-10
                     [    9.4742E-5 ,    -1.2583E-5,      -6.4928E-8,      1.0515E-8,     -2.0142E-10 ],
                     #A20 -3.9064E-7 A21  9.1061E-9  A22  -1.6009E-10 A23  7.994E-12 
                     [    -3.9064E-7,    9.1061E-9 ,      -1.6009E-10,     7.994E-12 ],
                     #A30 1.100E-10  A31  6.651E-12  A32  -3.391E-13
                     [    1.100E-10 ,    6.651E-12 ,      -3.391E-13 ] ]
            #
            coef_B = [ #B00  -1.922E-2 B01  -4.42E-5
                    [     -1.922E-2,     -4.42E-5],
                    #B10  7.3637E-5 B11  1.7950E-7
                    [      7.3637E-5,     1.7950E-7]]
            #
            coef_C= [ #C00 1402.388    C01 5.03830     C02 -5.81090E-2  C03 3.3432E-4   C04 -1.47797E-6 C05 3.1419E-9
                    [    1402.388  ,     5.03830   ,     -5.81090E-2,     3.3432E-4  ,    -1.47797E-6,    3.1419E-9],
                    #C10 0.153563    C11 6.8999E-4   C12 -8.1829E-6   C13 1.3632E-7   C14 -6.1260E-10 
                    [    0.153563  ,     6.8999E-4 ,     -8.1829E-6 ,     1.3632E-7  ,    -6.1260E-10],
                    #C20 3.1260E-5   C21 -1.7111E-6  C22 2.5986E-8    C23 -2.5353E-10 C24 1.0415E-12 
                    [    3.1260E-5 ,     -1.7111E-6,     2.5986E-8  ,     -2.5353E-10,    1.0415E-12] ,
                    #C30 -9.7729E-9  C31 3.8513E-10  C32 -2.3654E-12 
                    [    -9.7729E-9,     3.8513E-10,     -2.3654E-12] ] 
            #
            coef_D= [ #D00  1.727E-3
                    [1.727E-3],
                    #D10  -7.9836E-6
                    [-7.9836E-6]]
            #
            """Cw(T,P) = (C00 + C01T + C02T² + C03T³ + C04T⁴ + C05T5) +
                        (C10 + C11T + C12T² + C13T³ + C14T⁴)P +
                        (C20 + C21T +C22T² + C23T³ + C24T⁴)P² +
                        (C30 + C31T + C32T²)P³"""
            equacao_cw= ( coef_C[0][0] + coef_C[0][1] * temperatura + coef_C[0][2] * pow(temperatura,2) + coef_C[0][3] * pow(temperatura,3) + coef_C[0][4] * pow(temperatura,4) + coef_C[0][5] * pow(temperatura,5)) + \
                        ( coef_C[1][0] + coef_C[1][1] * temperatura + coef_C[1][2] * pow(temperatura,2) + coef_C[1][3] * pow(temperatura,3) + coef_C[1][4] * pow(temperatura,4)) * pressao + \
                        ( coef_C[2][0] + coef_C[2][1] * temperatura + coef_C[2][2] * pow(temperatura,2) + coef_C[2][3] * pow(temperatura,3) + coef_C[2][4] * pow(temperatura,4))* pow(pressao,2) + \
                        ( coef_C[3][0] + coef_C[3][1] * temperatura + coef_C[3][2] * pow(temperatura,2)) * pow(pressao,3)
            #
            """A(T,P) = (A00 + A01T + A02T² + A03T³ + A04T⁴) +
                    (A10 + A11T + A12T² + A13T³ + A14T⁴)P +
                    (A20 + A21T + A22T² + A23T³)P² +
                    (A30 + A31T + A32T²)P³"""
            equacao_a  = ( coef_A[0][0] + coef_A[0][1] * temperatura + coef_A[0][2] * pow(temperatura,2) + coef_A[0][3] * pow(temperatura,3) + coef_A[0][4] * pow(temperatura,4)) +\
                         ( coef_A[1][0] + coef_A[1][1] * temperatura + coef_A[1][2] * pow(temperatura,2) + coef_A[1][3] * pow(temperatura,3) + coef_A[1][4] * pow(temperatura,4)) * pressao +\
                         ( coef_A[2][0] + coef_A[2][1] * temperatura + coef_A[2][2] * pow(temperatura,2) + coef_A[2][3] * pow(temperatura,3)) * pow(pressao,2) +\
                         ( coef_A[3][0] + coef_A[3][1] * temperatura + coef_A[3][2] * pow(temperatura,2)) * pow(pressao,3)
            #
            """B(T,P) = B00 + B01T + (B10 + B11T)P"""
            equacao_b = coef_B[0][0] + coef_B[0][1] * temperatura + (coef_B[1][0] + coef_B[1][1] * temperatura) * pressao
            #
            """D(T,P) = D00 + D10P"""
            equacao_d = coef_D[0][0] + coef_D[1][0] * pressao
            # 
            # c(S,T,P) = Cw(T,P) + A(T,P)S + B(T,P)S3/2 + D(T,P)S2
            resultado_equacao = equacao_cw + equacao_a * salinidade + equacao_b * pow(salinidade,3/2) + equacao_d * pow( salinidade,2)
            #
            return resultado_equacao
    
    def calcular_equacao_del_grosso(self, temperatura, salinidade, pressao):
        """
            Uma equação alternativa ao algoritmo da UNESCO, que tem uma gama mais restrita de validade, 
            é a equação de Del Grosso (1974). 
            Faixa de validade: temperatura de 0 a 30°C, salinidade de 30 a 40 partes por mil, pressão de 0 a 1000 kg/cm², 
            onde 100 kPa=1,019716 kg/cm2. (Wong e Zhu, 1995).
            link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_delgrosso
        Args:
            temperatura (float): temperatura em graus Celsius (0 a 30°C)
            salinidade (float): salinidade em partes por mil (30 a 40)
            pressao (float): pressão em Quilopascal  (0 a 98066 Kpa)
        Returns
            velocidade_som(float): Velocidade do som na agua             
        Atenção: Essa função apresenta erro na 8ª casa decimal
        """

        temperatura = float(temperatura)
        salinidade = float(salinidade)
        pressao = float(pressao) / 98.066 #em kg/cm²
        #
        if not (temperatura >= 0 and temperatura <= 30):
            raise Exception(
                'Informe uma Temperatura válida entre 0 a 30 graus Celsius')
        #
        if not (salinidade >= 30 and salinidade <= 40):
            raise Exception(
                'Informe uma salinidade válida entre 30 a 40 partes por mil')
        #
        if not (pressao >= 0 and pressao <= 1000):
            raise Exception(
                'Informe uma pressão válida entre 0 a 98066 KPas ou 1000 kg/cm²')
        #
        # C000  1402.392 
        coef_C000 = 1402.392 
        # CT1  0.5012285E1 CT2  -0.551184E-1 CT3  0.221649E-3
        coef_CT = [ 0.5012285E1, -0.551184E-1, 0.221649E-3 ]
        # CS1  0.1329530E1 CS2  0.1288598E-3 
        coef_CS = [ 0.1329530E1, 0.1288598E-3 ]
        # CP1  0.1560592   CP2  0.2449993E-4 CP3  -0.8833959E-8
        coef_CP = [ 0.1560592  , 0.2449993E-4, -0.8833959E-8 ]
        # CST  -0.1275936E-1
        coef_CST = -0.1275936E-1
        # CTP  0.6353509E-2
        coef_CTP = 0.6353509E-2
        # CT2P2  0.2656174E-7
        coef_CT2P2 = 0.2656174E-7
        # CTP2  -0.1593895E-5 
        coef_CTP2 = -0.1593895E-5 
        # CTP3  0.5222483E-9 
        coef_CTP3 = 0.5222483E-9 
        # CT3P  -0.4383615E-6
        coef_CT3P = -0.4383615E-6
        # CS2P2  -0.1616745E-8
        coef_CS2P2 = -0.1616745E-8
        # CST2  0.9688441E-4
        coef_CST2 = 0.9688441E-4
        # CS2TP  0.4857614E-5
        coef_CS2TP = 0.4857614E-5
        # CSTP  -0.3406824E-3
        coef_CSTP = -0.3406824E-3
        
        #ΔCT(T) = CT1T + CT2T² + CT3T³
        equacao_CT = coef_CT[0] * temperatura + coef_CT[1] * pow(temperatura,2) + coef_CT[2] * pow(temperatura,3)
        #ΔCS(S) = CS1S + CS2S²
        equacao_CS = coef_CS[0] * salinidade + coef_CS[1] * pow(salinidade,2)
        #ΔCP(P) = CP1P + CP2P² + CP3P³
        equacao_CP = coef_CP[0] * pressao + coef_CP[1] * pow(pressao,2) + coef_CP[2] * pow(pressao,3)
        #ΔCSTP(S,T,P) = CTPTP + CT3PT³P + CTP2TP² + CT2P2T²P² + CTP3TP³ + CSTST + CST2ST² + CSTPSTP + CS2TPS²TP + CS2P2S²P²
        equacao_CSTP =   coef_CTP * temperatura * pressao + coef_CT3P * pow(temperatura,3) * pressao \
                       + coef_CTP2 * temperatura * pow(pressao,2) + coef_CT2P2 * pow(temperatura,2) * pow(pressao,2) \
                       + coef_CTP3 * temperatura * pow(pressao,3) + coef_CST * salinidade * temperatura \
                       + coef_CST2 * salinidade * pow(temperatura,2) + coef_CSTP * salinidade * temperatura * pressao \
                       + coef_CS2TP * pow(salinidade,2) * temperatura * pressao  + coef_CS2P2 * pow(salinidade,2) * pow(pressao,2)
        #
        # c(S,T,P) = C000 + ΔCT + ΔCS + ΔCP + ΔCSTP
        resultado_equacao = coef_C000 + equacao_CT + equacao_CS + equacao_CP + equacao_CSTP
        #
        return resultado_equacao
    
    def calcular_equacao_NPL(self, T, S, profundidade, latitude):        
        """
            Equação adequada para uso em todas as águas "neptunianas", excluindo os "pontos quentes" anormais 
            de temperatura e salinidade anormalmente altas, a fim de permitir o calculo com precisão
            a velocidade do som em vários cenários usando apenas uma única equação
            Faixa de validade: Esta equação é válida para uso em qualquer oceano ou mar com uma salinidade que não exceda 42.
            link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_npl
        Args:
            temperatura (float): temperatura em graus Celsius
            salinidade (float): salinidade em partes por mil (menor que 42)
            profundidade (float): profundidade em metros
            latitude(float): latitude em graus 
        Returns
            velocidade_som (float): Velocidade do som na agua             
        """
        pass
        # T = float(T)
        # S = float(S)
        # profundidade = float(profundidade)
        # latitude =  float(latitude)
        # #
        # if not (S >= 0 and S <= 42):
        #     raise Exception(
        #         'Informe uma salinidade válida 0 a 42 partes por mil')
        # #
        # """c = 1402.5 + 5T - 5.44 x 10⁻²T + 2.1 x 10⁻⁴T³
        #     + 1.33S - 1.23 x 10⁻²ST + 8.7 x 10⁻⁵ST²
        #     +1.56 x 10⁻²Z + 2.55 x 10⁻⁷Z² - 7.3 x 10⁻¹²Z³
        #     + 1.2 x 10⁻⁶Z(Φ - 45) - 9.5 x 10⁻¹³TZ³
        #     + 3 x 10⁻⁷T²Z + 1.43 x 10⁻⁵SZ"""
        # Z=0
        # resultado_equacao = 1402.5 + 5 * T - 5.44 * pow(10,-2) * T + 2.1 * pow(10,-4) * pow(T,3) \
        #                     + 1.33 * S - 1.23 * pow(10,-2) * S * T + 8.7 * pow(10,-5)*S* pow(T,2) \
        #                     + 1.56 * pow(10,-2) * Z + 2.55 * pow(10,-7) * pow(Z,2) - 7.3 * pow(10,-12)* pow(Z,3) \
        #                     + 1.2 * pow(10,-6) * Z * (latitude - 45) - 9.5 * pow(10,-13) * T* pow(Z,3) \
        #                     + 3 * pow(10,-7) * pow(T,2) * Z + 1.43 * pow(10,-5) * S * Z

        # #
        # return resultado_equacao

    def calcular_distancia_3D(self, coordenadada_origem, coordenadada_destino):
        """
            Calcula a distancia 3D entre dois pontos
        Args:
            coordenadada_origem ([tuple(x,y,z)]): Coordenada X,Y e Z Origem
            coordenadada_destino ([tuple(x,y,z)]): Coordenada X,Y e Z Destino

        Returns:
            [float]: [distancia entre dois pontos]
        """
        x_origem, y_origem, z_origem = coordenadada_origem
        x_destino, y_destino, z_destino = coordenadada_destino
        distancia = math.sqrt(pow((x_destino - x_origem),2) + pow((y_destino - y_origem),2) + pow((z_destino - z_origem),2))
        return distancia
    
    def calcular_delay_transmissao(self,coordenadada_origem, coordenadada_destino,
                                        tipoequacao = None, 
                                        temperatura = None,salinidade= None, 
                                        pressao     = None,profundidade=None, latitude=None):
        #
        tipoequacao  = tipoequacao  or self.tipoequacao  # isso é tipo um null coalescing do python, se esquerda fo nula rebece a direita
        temperatura  = temperatura  or self.temperatura 
        salinidade   = salinidade   or self.salinidade 
        pressao      = pressao      or self.pressao 
        profundidade = profundidade or self.profundidade 
        latitude     = latitude     or self.latitude
        # 
        velocidade_som = self.calcular_velocidade_som(tipoequacao, temperatura, salinidade, pressao, profundidade, latitude)
        distancia = self.calcular_distancia_3D(coordenadada_origem,coordenadada_destino)
        delay = velocidade_som * distancia
        #
        return delay 

        

# class ConfiguracaoRede():
    
#     def __init__(self, conf_padrao=True, dict_conf_rede = {}, end_arquivo_config=""):
#         self.conf_rede_atual =  dict_conf_rede
#         #
#         if end_arquivo_config != "":
#             self.conf_rede_atual = self.abrir_arquivo_config(end_arquivo_config)
#         elif conf_padrao == True and dict_conf_rede == {}:
#             self.conf_rede_atual = self.gerar_config_rede()

#     @property
#     def data_packet_length(self):
#         return self.conf_rede_atual['DataPacketLength']

#     @data_packet_length.setter
#     def data_packet_length(self, tam_pacote_mensagem):
#         self.conf_rede_atual['DataPacketLength'] = tam_pacote_mensagem    

#     @property
#     def posicao_node_destino(self):
#         return self.conf_rede_atual['Nodes'][0] [1]

#     @posicao_node_destino.setter
#     def posicao_node_destino(self, tpl_pos_node_destino):
        
#             Alterar a posição do remente no grid
#         Args:
#             tpl_pos_node_remetente (tuple()): tupla no formato(x, y z)
#         """
#         self.conf_rede_atual['Nodes'][0] [1] = tpl_pos_node_destino

#     @property
#     def posicao_node_remente(self):
#         """
#             Retorna a posição do remente no grid
#         Returns:
#             [float]: posição do remente no grid
#         """
#         return self.conf_rede_atual['Nodes'][1] [1]

#     @posicao_node_remente.setter
#     def posicao_node_remente(self, tpl_pos_node_remetente):
#         """
#             Alterar a posição do remente no grid
#         Args:
#             tpl_pos_node_remetente (tuple()): tupla no formato(x, y z)
#         """
#         self.conf_rede_atual['Nodes'][1] [1] = tpl_pos_node_remetente

#     def gerar_config_rede(self, SimulationDuration = 3600.00, 
#                                 BandWidth =  10.89, 
#                                 BandwidthBitrateRelation =   1.00,
#                                 Frequency =   9.22, 
#                                 TransmitPower = 250.00 ,
#                                 ReceivePower = -30.00 ,
#                                 ListenPower = -30.00 ,
#                                 DataPacketLength = 9600.00, #bits 
#                                 PHY = {"SIRThreshold": 15.00, "SNRThreshold": 20.00, "LISThreshold":  3.00, "variablePower":True, "variableBandwidth":False,"level2distance":{0:11224.97}},
#                                 MAC = {"protocol":"ALOHA", "max2resend":10.0, "attempts":4, "ACK_packet_length":24, "RTS_packet_length":48, "CTS_packet_length":48, "WAR_packet_length":24, "SIL_packet_length":24, "tmin/T":2.0, "twmin/T":0.0, "deltatdata":0.0, "deltad/T":0.0, },
#                                 Routing = {"Algorithm": "Static", "variation":2, "coneAngle":60, "coneRadius":11174.97, "maxDistance":10e3},
#                                 Nodes = [["SinkBoia", (5000,5000,500), None, "Submarino"], ["Submarino",(5000,15000,500), None, "SinkBoia"]],
#                                 NodeField = (5000.00,4,4, (0,0,0), "S"),
#                                 Height = 1000.00,
#                                 Period = 240.0,
#                                 RandomGrid = False,
#                                 Moving = False,
#                                 Speed = 0.0):
#         """
#             Retorna as configurações padrão da rede subaquatica 
#         Args:
#             SimulationDuration (float, optional): Duracao da simulacao. Defaults to 3600.00.
#             BandWidth (float, optional): Largura de banda disponivel (kHz). Defaults to 10.89.
#             BandwidthBitrateRelation (float, optional): Eficiencia de largura de banda. Defaults to 1.00.
#             Frequency (float, optional): Frequencia (kHz). Defaults to 9.22.
#             TransmitPower (float, optional): Potencia maxima de transmissao -> intensidade acustica (dB re uPa). Defaults to 250.00.
#             ReceivePower (float, optional): Consumo de energia para receber mensagem (dB) -> consumo de bateria (dB). Defaults to -30.00.
#             ListenPower (float, optional): Consumo de energia para Ouvir mensagem (dB) -> consumo de bateria (dB W). Defaults to -30.00.
#             DataPacketLength (float, optional): Tamanho do pacote de dados. Defaults to 9600.00.
#             PHY (dict, optional): PHY: Parâmetros para camada fisica.Defaults to {"SIRThreshold": 15.00, "SNRThreshold": 20.00, "LISThreshold":  3.00, "variablePower":True, "variableBandwidth":False,"level2distance":{0:11224.97}}
#             MAC (dict, optional): MAC: Definir o protocolo e os seus parametros da camada MAC. Defaults to {"protocol":"ALOHA", "max2resend":10.0, "attempts":4, "ACK_packet_length":24, "RTS_packet_length":48, "CTS_packet_length":48, "WAR_packet_length":24, "SIL_packet_length":24, "tmin/T":2.0, "twmin/T":0.0, "deltatdata":0.0, "deltad/T":0.0, }.
#             Routing (dict, optional): Roteamento: defina parametros para o Algorimo de roteamento. Defaults to {"Algorithm": "Static", "variation":2, "coneAngle":60, "coneRadius":11174.97, "maxDistance":10e3}.
#             Nodes (list, optional): Configuração dos nós --> Formato: AcousticNode(Address, position[, period, destination]). Defaults to [ ["SinkMobile", [[(3000,-3000, 1000),(3000,3000,500),(-3000,3000,1000)],1.0], 300, "StaticNode013"]].
#             NodeField (tuple, optional): Configuração dos campos do nós -- Formato (grid_block_size, N_blocks_wide, N_blocks_high[, bottom_left_corner[, node_ID_prefix]). Defaults to (5000.00,4,4, (0,0,0), "S").
#             Height (float, optional): Altura maxima/profundidade em que os nos podem está. Defaults to 1000.00.
#             Period (float, optional): Periodo de envio de mensagem. Defaults to 240.0.
#             RandomGrid (bool, optional): Indica se é para gerar grid aleatória para os nós . Defaults to False.
#             Moving (bool, optional): Indica se é para gerar grid aleatória para os nós. Defaults to False.
#             Speed (float, optional): Velocidade de movimentação. Defaults to 0.0.

#         Returns:
#             dictionary: Configuração da rede
#         """
#         conf_rede = {}
#         # Simulation Duration (seconds)
#         # Duracao da simulacao (segundos)
#         conf_rede["SimulationDuration"]       =SimulationDuration        
#         # Available Bandwidth (kHz) 
#         # Largura de banda disponivel (kHz)
#         conf_rede["BandWidth"]                =BandWidth                
#         # Bandwidth and bitrate relation (bps/Hz) 
#         #  Eficiencia de largura de banda (bps/Hz)
#         conf_rede["BandwidthBitrateRelation"] =BandwidthBitrateRelation 
#         # Frequency (kHz) 
#         # Frequencia (kHz)
#         conf_rede["Frequency"]                =Frequency                
#         # Transmit Power -> Acoustic Intensity (dB re uPa) -> Default value 
#         # Potencia maxima de transmissao -> intensidade acustica (dB re uPa)
#         conf_rede["TransmitPower"]            =TransmitPower            
#         # Receive Power (dB) -> Battery Consumption (dB) -> 3W for PSK modulation and 80mW for FSK
#         # Consumo de energia para receber mensagem (dB) -> consumo de bateria (dB)
#         conf_rede["ReceivePower"]             =ReceivePower             
#         # Listen Power (dB) -> Battery Consumption (dB) -> 80mW 
#         # Consumo de energia para Ouvir mensagem (dB) -> consumo de bateria (dB W)
#         conf_rede["ListenPower"]              =ListenPower              
#         # DataPacketLength (bits) 
#         #Tamanho do pacote de dados
#         conf_rede["DataPacketLength"]         =DataPacketLength         
#         # PHY: set parameters for the physical layer 
#         #PHY: Parâmetros para camada fisica
#         conf_rede["PHY"]                      =PHY                      
#         # MAC: define which protocol we are using & set params
#         #MAC: Definir o protocolo e os seus parametros da camada MAC
#         conf_rede["MAC"]                      =MAC                      
#         # Routing: set parameters for the routing layer
#         # Roteamento: defina parametros para o Algorimo de roteamento
#         conf_rede["Routing"]                  =Routing                  
#         # Nodes: here is where we define individual nodes
#         #Nodes: aqui e onde definimos nos individuais
#         # format: AcousticNode(Address, position[, period, destination])
#         # Nodes -> Formato: Endereco, Posicao [, periodo, destino]
#         conf_rede["Nodes"]                    =Nodes                    
#         # NodeField: Set up a field of nodes
#         # NodeField: Configure um campo de nos.
#         # format (grid_block_size, N_blocks_wide, N_blocks_high[, bottom_left_corner[, node_ID_prefix])
#         conf_rede["NodeField"]                =NodeField                
#         # Maximum height/depth at which nodes can be
#         # Altura maxima/profundidade em que os nos podem está
#         conf_rede["Height"]                   =Height                   
#         # By default, the nodes in the node field are just relays, but we can make them also generate information by changing this value.
#         # Por padrao, os nos no campo de no sao apenas reles, mas podemos faze-los tambem gerar informacoes alterando esse valor
#         conf_rede["Period"]                   =Period                   
#         # It makes more sense to place nodes randomly rather than in a perfect grid
#         # Faz mais sentido colocar nos aleatoriamente em vez de em uma grade
#         conf_rede["RandomGrid"]               =RandomGrid               
#         # Nodes in the node field can be moving randomly and in without being aware of it because of underwater flows.
#         # Nos no campo de no podem estar se movendo aleatoriamente e dentro sem estar ciente disso por causa dos fluxos subaquatico
#         conf_rede["Moving"]                   =Moving                   
#         # velocidade de movimentacao na agua
#         conf_rede["Speed"]                    =Speed    
#         #
#         return conf_rede 

#     def abrir_arquivo_config(self, end_arquivo_config):
#         end_arquivo_config = sys.argv[1] if (len(sys.argv) == 2) else end_arquivo_config
#         #
#         if end_arquivo_config is None or end_arquivo_config == '':
#             print('Endereço do arquivo de configuração NÃO INFORMADO. Nem no Argv e nem no Parâmetro')
#             return {}
#         #
#         conf_rede_lida = AUVSim.ReadConfigFromFile(end_arquivo_config)
#         self.conf_rede_atual = conf_rede_lida
#         #
#         return conf_rede_lida          


# class BrovComunicacao():
    
#     def __init__(self, dict_config_rede_subaquatica={}):
#         self.config_rede_subaquatica = ConfiguracaoRede(dict_conf_rede=dict_config_rede_subaquatica)

#     # Obtains the average delay and energy consumption per consumption
#     def GetStat(self, nodes):
#         packets = 0
#         delay = []
#         energy_vec = []
#         collisions = 0
        
#         for x in nodes:
#             if x.physical_layer.tx_energy+x.physical_layer.rx_energy!=0:
#                 energy_vec.append(x.physical_layer.tx_energy+x.physical_layer.rx_energy)

#             if len(x.app_layer.packets_time)!=0:
#                 packets+=len(x.app_layer.packets_time)
#                 delay.append(self.Average(x.app_layer.packets_time))

#             if len(x.physical_layer.transducer.collisions)!=0:
#                 collisions+=len(x.physical_layer.transducer.collisions)

#         energy = sum(energy_vec)
#         delay = self.Average(delay)

#         return energy, packets, delay, collisions

#     # Returns the average tx level used
#     def GetLevel(self,nodes):
#         lev = []
#         for x in nodes:
#             if len(x.routing_layer.levels_used):
#                 lev.append(self.Average(x.routing_layer.levels_used))

#         #print ("Average of the output power level covering:", self.Average(lev), "m.")
#         return self.Average(lev)
    
#     # Computes the network throughput
#     def Throughput(self,nodes, simtime):
#         generated_packets = 0
#         received_packets = 0
#         delay = []
#         hops = []
        
#         for x in nodes:
#             generated_packets+=x.app_layer.packets_sent
#             received_packets+=len(x.app_layer.packets_time)

#             if len(x.app_layer.packets_time)!=0:
#                 delay.append(self.Average(x.app_layer.packets_time))
#                 hops.append(self.Average(x.app_layer.packets_hops))

#         t_data = nodes[0].MACProtocol.t_data

#         throughput = received_packets*t_data/simtime*self.Average(hops)
#         offeredload = generated_packets*t_data/simtime*self.Average(hops)

#         ##    print "The throughput achieved for an offered load of", offeredload, "is", throughput
#         return offeredload, throughput

#     # Plots the scenario where the nodes are currently located. Size is proportional to transmited energy, color changes with reception power
#     def PlotScenario(self,nodes):
#         tx_energy_vec = []
#         rx_energy_vec = []
#         xpos = []
#         ypos = []
        
#         pylab.figure()
#         pylab.hold(True)

#         for x in nodes:
#             xpos.append(x.GetCurrentPosition()[0])
#             ypos.append(x.GetCurrentPosition()[1])

#             tx_energy_vec.append(x.physical_layer.tx_energy)
#             rx_energy_vec.append(x.physical_layer.rx_energy)

#             if x.physical_layer.tx_energy > 1.0 or x.name[0:4] == "Sink":
#                 pylab.text(x.GetCurrentPosition()[0],x.GetCurrentPosition()[1],x.name)

#         self.Normalize(tx_energy_vec, max(tx_energy_vec))
#         self.Normalize(rx_energy_vec, max(rx_energy_vec))
        
#         pylab.scatter(xpos, ypos, s=tx_energy_vec, c=rx_energy_vec)
#         pylab.xlabel('X(m)')
#         pylab.ylabel('Y(m)')
#         pylab.title('Scenario')

#     # Plots the scenario in 3D axis
#     def PlotScenario3D(self,nodes, config):
#         # import matplotlib.axes as p3
#         from mpl_toolkits.mplot3d import Axes3D

#         fig=pylab.figure()
#         ax = Axes3D(fig)

#         filename = 'Positions_'+config["MAC"]["protocol"]+'_'+str(config["Routing"]["Algorithm"])+'.txt'
#         f = open(filename,'w')
#         filename = 'Routes_'+config["MAC"]["protocol"]+'_'+str(config["Routing"]["Algorithm"])+'.txt'
#         ff = open(filename,'w')
#         ff.write('-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n')

#         positions=[]
#         tx_energy_vec = []
#         rx_energy_vec = []    
        
#         for node in nodes:
#             pos=node.GetCurrentPosition()
#             positions.append(pos)       
#             tx_energy_vec.append(node.physical_layer.tx_energy)
#             rx_energy_vec.append(node.physical_layer.rx_energy)
#             record = str(pos[0])+', '+str(pos[1])+', '+str(pos[2])+', '+str(node.physical_layer.tx_energy)+', '+str(node.physical_layer.rx_energy)+'\n'
#             f.write(record)

#             for log_line in node.app_layer.log:
#                 for i in log_line["route"]:
#                     route = i[1]
#                     ff.write(str(route[0])+', '+str(route[1])+', '+str(route[2])+', ')
#                 ff.write(str(pos[0])+', '+str(pos[1])+', '+str(pos[2])+', -1\n')

#                 route = [i[1] for i in log_line["route"]]
#                 route.append(pos)
#                 rx,ry,rz = self.ReorderPositions(route)
#                 ax.plot3D(rx,ry,rz)

#             # pylab.text(10,20,"max" )
#             # pylab.text(x=node.GetCurrentPosition()[0],y=node.GetCurrentPosition()[1],s=node.name)

#         f.close()
#         self.Normalize(tx_energy_vec, max(tx_energy_vec))
#         self.Normalize(rx_energy_vec, max(rx_energy_vec))

#         nx,ny,nz = self.ReorderPositions(positions)
#         ax.scatter3D(nx,ny,nz,s=tx_energy_vec, c=rx_energy_vec)

#         ax.set_xlabel('X [m]')
#         ax.set_ylabel('Y [m]')
#         ax.set_zlabel('Z [m]')
#         ax.axis("equal")

#     def ReorderPositions(self,PositionContainer):
#         """Reorder an array (list or tuple) of positions tuples into 3 lists:
#         a list of x's,y's, and z's for plotting.
#         """
#         x=[]
#         y=[]
#         z=[]
#         for pos in PositionContainer:
#             x.append(pos[0])
#             y.append(pos[1])
#             z.append(pos[2])
    
#         return (x,y,z)

#     # Plots the energy consumption per node both in reception and in transmission
#     def PlotConsumption(self,nodes):
#         tx_energy_vec = []
#         rx_energy_vec = []

#         pylab.figure()

#         for x in nodes:
#             tx_energy_vec.append(x.physical_layer.tx_energy)
#             rx_energy_vec.append(x.physical_layer.rx_energy)

#         N = len(tx_energy_vec)
#         ind = pylab.arange(N)   # the x locations for the groups
#         width = 1               # the width of the bars
#         p1 = pylab.bar(ind, tx_energy_vec, width, color='r')
#         p2 = pylab.bar(ind, rx_energy_vec, width, color='g', bottom=tx_energy_vec)

#         pylab.ylabel('Energy (J)')
#         pylab.title('Power Consumption')

#         pylab.legend( (p1[0], p2[0]), ('Tx', 'Rx') )

#     # Plots the histogram of the consumed energy all over the network
#     def PlotConsumptionHist(self,nodes):
#         tx_energy_vec = []
#         rx_energy_vec = []
#         energy_vec = []

#         for x in nodes:
#             if x.physical_layer.tx_energy!=0:
#                 tx_energy_vec.append(x.physical_layer.tx_energy)        
#             rx_energy_vec.append(x.physical_layer.rx_energy)
#             energy_vec.append(x.physical_layer.tx_energy+x.physical_layer.rx_energy)

#         energy = self.Average(energy_vec)
#         print ("Network total energy consumption:", sum(tx_energy_vec), "and average consumption per active node:", energy, "J." )
#         return energy

#         pylab.figure()    
#         pylab.hist(tx_energy_vec,20);
#         pylab.xlabel("Energy (J)");
#         pylab.ylabel("Number of nodes");
#         title = "Transmitting energy consumption"
#         pylab.title(title)

#         pylab.figure()
#         pylab.hist(rx_energy_vec,20);    
#         pylab.xlabel("Energy (J)");
#         pylab.ylabel("Number of nodes");
#         title = "Receiving energy consumption"
#         pylab.title(title)

#     # Plot the total delay and the number of hops for each packet in each node
#     def PlotDelay(self,nodes):
#         av = []
#         name = []
#         pylab.figure()
#         width = 0.5
        
#         for x in nodes:
#             if len(x.app_layer.packets_time)!=0:
#                 av.append(self.Average(x.app_layer.packets_time))
#                 name.append(x.name)

#         ind = pylab.arange(len(av))
#         pylab.bar(ind, av, width)
#         pylab.xticks(ind+width/2, name)
        
#     # Plot an histogram for each sink showing packets' delay
#     def PlotDelayHist(self,nodes):
#         packets = 0
#         delay = []
        
#         for x in nodes:
#             if len(x.app_layer.packets_time)!=0:
#                 print ("Total number of packets received at", x.name, ":", len(x.app_layer.packets_time))
#                 packets+=len(x.app_layer.packets_time)
#                 delay.append(self.Average(x.app_layer.packets_time))

#                 print (x.app_layer.packets_time)
#                 pylab.figure()
#                 pylab.hist(x.app_layer.packets_time,100);
#                 pylab.xlabel("End to End delay (s)");
#                 pylab.ylabel("Number of Packets");
#                 title = "End to End Delay for packets received at "+x.name
#                 pylab.title(title)

#                 pylab.figure()
#                 pylab.hist(x.app_layer.packets_dhops,10);
#                 pylab.xlabel("Delay per hop (s)");
#                 pylab.ylabel("Number of Packets");
#                 title = "Delay per hop for packets received at "+x.name
#                 pylab.title(title)

#         print ("Average delay per packet in the network:", self.Average(delay), "s.")
#         return packets, delay
        
#     # Normalizes energy in a scale from 10 to 1000 to achieve proper plots    
#     def Normalize(self,x, y):
#         for i in range(0,len(x)):
#             x[i]=x[i]/y*1000+50

#     # Computes the average value of vector x
#     def Average(self,x):
#         av = 0
#         for i in range(0,len(x)):
#             av+=x[i]

#         return av/len(x)
   
#     def ROU(self,end_arquivo_config=None):

#         end_arquivo_config = sys.argv[1] if (len(sys.argv) == 2) else end_arquivo_config
#         if end_arquivo_config is None:
#             print('Endereço do arquivo de configuração NÃO INFORMADO. Nem no Argv e nem no Parâmetro')
#             exit(1) 

#         config = AUVSim.ReadConfigFromFile(end_arquivo_config)

#         random.seed()
        
#         def GenerateRandomPeriodicTransmit(min, max):
#             return random.random()*(max-min) + min

#         period = []
#         energy = []
#         packets = []
#         delay = []
#         offered_load = []
#         throughput = []
#         to = []
#         col = []
#         output_power = []
#         t_data = []

#         for j in range(0,1):
#             filename = 'sim_N_'+str(config["NodeField"][1])+'_f_'+str(config["Frequency"])+'_b_'+str(config["BandWidth"])+'_'+config["MAC"]["protocol"]+'_'+config["Routing"]["Algorithm"]+'.txt'
#             f = open(filename,'w')

#             period.append([])
#             energy.append([])
#             packets.append([])
#             delay.append([])
#             offered_load.append([])
#             throughput.append([])
#             to.append([])
#             col.append([])
#             output_power.append([])
#             t_data.append([])

#             for i in range(0,1):
#                 period[j].append(config["Period"])
#                 energy_s = []
#                 packets_s = []
#                 delay_s = []            
#                 col_s = []
#                 offered_load_s = []
#                 throughput_s = []
#                 to_s = []

#                 output_power_s = []
#                 t_data_s = []
                
#                 for z in range(0,10):
#                     nodess = AUVSim.RunSimulation(config)
#                     nodes=[]
#                     for x in nodess:
#                         nodes.append(nodess[x])                

#                     energy_ss, packets_ss, delay_ss, col_ss = self.GetStat(nodes)
#                     offered_load_ss, throughput_ss = self.Throughput(nodes, config["SimulationDuration"])
#                     energy_s.append(energy_ss)
#                     packets_s.append(packets_ss)
#                     delay_s.append(delay_ss)
#                     col_s.append(col_ss)
#                     offered_load_s.append(offered_load_ss)
#                     throughput_s.append(throughput_ss)
#                     to_s.append(throughput_ss/offered_load_ss)

#                     output_power_ss = []
#                     for x in nodes:
#                         if x.physical_layer.max_output_power!=0:
#                             output_power_ss.append(x.physical_layer.max_output_power_used)

#                     output_power_s.append(max(output_power_ss))
#                     t_data_s.append(x.MACProtocol.t_data)

#                 energy[j].append(self.Average(energy_s))
#                 packets[j].append(self.Average(packets_s))
#                 delay[j].append(self.Average(delay_s))
#                 col[j].append(self.Average(col_s))
#                 throughput[j].append(self.Average(throughput_s))
#                 offered_load[j].append(self.Average(offered_load_s))
#                 to[j].append(self.Average(to_s))
#                 output_power[j].append(max(output_power_s))
#                 t_data[j].append(self.Average(t_data_s))

#                 # File recordings
#                 record = str(config["Period"])+', '+str(delay[j][i])+', '+str(energy[j][i])+', '+str(offered_load[j][i])
#                 record = record+', '+str(throughput[j][i])+', '+str(col[j][i])+', '+str(packets[j][i])+', '+str(output_power[j][i])+'\n'
#                 f.write(record)

#             print (config["PHY"]["level2distance"], config["Frequency"], config["BandWidth"])
#             f.close() 
    
#     def inicializaSimulacao(self,end_arquivo_config=None):
#         end_arquivo_config = sys.argv[1] if (len(sys.argv) == 2) else end_arquivo_config
#         if end_arquivo_config is None:
#             print('Endereço do arquivo de configuração NÃO INFORMADO. Nem no Argv e nem no Parâmetro')
#             exit(1)  
#         #
#         config_simulacao = AUVSim.ReadConfigFromFile(end_arquivo_config)
#         #
#         print('Inicializando Simulação')
#         nodess = AUVSim.RunSimulation(config_simulacao)
#         print('Simulação finalizada')
#         #
#         nodes = []
#         for node in nodess:
#             nodes.append(nodess[node]) 
        
#         energy, packets, delay, col = self.GetStat(nodes)

#         print ("Energy consumption: ",energy)
#         print( "Total number of packets transmitted: ",packets)
#         print ("Average end to end delay: ",delay)
#         print ("Collisions: ",col)

#         # self.PlotScenario3D(nodes, config_simulacao)
#         # self.PlotConsumption(nodes)
#         # self.PlotDelay(nodes)
#         # self.PlotDelayHist(nodes)    
#         # pylab.show()  

#     def enviar_mensagem(self, str_mensagem, posicao_remetente, posicao_destinatario):        
#         #
#         self.config_rede_subaquatica.data_packet_length = len(str_mensagem.encoder('uft8'))
#         self.config_rede_subaquatica.posicao_node_destino = posicao_destinatario
#         self.config_rede_subaquatica.posicao_node_remente = posicao_remetente
#         #
#         print('Inicializando Simulação')
#         nodess = AUVSim.RunSimulation(self.config_rede_subaquatica.conf_rede_atual)
#         print('Simulação finalizada')
#         #
#         nodes = []
#         for node in nodess:
#             nodes.append(nodess[node]) 
        
#         energy, packets, delay, col = self.GetStat(nodes)

#         print ("Energy consumption: ", energy)
#         print( "Total number of packets transmitted: ", packets)
#         print ("Average end to end delay: ", delay)
#         print ("Collisions: ", col)
#         delay = 0

#     @staticmethod
#     def inicializa_comunicacao():
#         nome_arquivo = '.\\ConfigComunicacaoBrov.conf'
#         # configuracaorede = ConfiguracaoRede()
#         brovcomunicacao = BrovComunicacao()
#         # cal = CalcularVelocidadeSomAgua()
#         brovcomunicacao.inicializaSimulacao(nome_arquivo)
#         # brovcomunicacao.ROU(nome_arquivo)

if __name__ == "__main__":
    nome_arquivo = '.\\ConfigComunicacaoBrov.conf'
    # BrovComunicacao.inicializa_comunicacao()
    # calcula_velocidade_som = CalcularVelocidadeSomAgua()
    # resutado_esperado = 1501.3503029378637
    # temperatura = 17
    # salinidade = 25
    # profundidade = 10
    # resultado= calcula_velocidade_som.calcular_equacao_mackenzie(temperatura, salinidade, profundidade)
    ##*********************Configuração da Transmissao*********************
    TIPO_EQUACAO =  TipoEquacao.MACKENZIE
    TEMPERATURA = 17
    SALINIDADE = 25
    PROFUNDIDADE = 10
    PRESSAO = None
    LATITUDE =  None
    ##*********************Configuração da Transmissao*********************
    coordenadada_origem_teste  = (2,3,5)
    coordenadada_destino_teste = (3,5,7)
    calcula_velocidade_som = CalcularVelocidadeSomAgua(tipoequacao = TIPO_EQUACAO,
                                                        temperatura=TEMPERATURA,
                                                        salinidade=SALINIDADE, 
                                                        profundidade=PROFUNDIDADE,
                                                        pressao=PRESSAO,latitude=LATITUDE )
    delay =  calcula_velocidade_som.calcular_delay_transmissao(coordenadada_origem_teste, coordenadada_destino_teste,
                                                               tipoequacao= TipoEquacao.COPPENS,
                                                               temperatura=10)#, salinidade= salinidade, profundidade=profundidade)
    print (delay)