class CalcularVelocidadeSomAgua():
    def Calcular_Velocidade(self,
                            temperatura,
                            salinidade,
                            pressao,
                            profundidade=0,
                            latitude=0):
        print("teste")

    def equacao_mackenzie(self, temperatura, salinidade, profundidade):
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
                'Informe uma profundidade válidaentre 0 a 8000 metros')
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

    def equacao_coppens(self, temperatura, salinidade, profundidade):
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

    def equacao_unesco(self, temperatura, salinidade, pressao):
            """
                O algoritmo padrão internacional, muitas vezes conhecido como algoritmo da UNESCO, é devido a Chen e Millero (1977), 
                usa a pressão como variável em vez de profundidade.
                Faixa de validade: temperatura de 0 a 40 °C, salinidade de 0 a 40 partes por mil, pressão de 0 a 1000 bar (Wong e Zhu, 1995).
                link: http://resource.npl.co.uk/acoustics/techguides/soundseawater/underlying-phys.html#up_unesco
            Args:
                temperatura (float): temperatura em graus Celsius (0 a 40°C)
                salinidade (float): salinidade em partes por mil  (0 a 40)
                pressao (float): pressão em Quilopascal  (0 a 100000 Kpa)
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

    def equacao_del_grosso(self, temperatura, salinidade, pressao):
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
        """

        temperatura = float(temperatura)
        salinidade = float(salinidade)
        pressao = float(pressao) #em kg/cm²
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
        equacao_CSTP =   coef_CTP * temperatura * pressao + coef_CT3P * pow(temperatura,3) * pressao + coef_CTP2 * temperatura * pow(pressao,2) \
                       + coef_CT2P2 * pow(temperatura,2) * pow(pressao,2) + coef_CTP3 * temperatura * pow(pressao,3) \
                       + coef_CST * salinidade * temperatura + coef_CST2 * salinidade * pow(temperatura,2) \
                       + coef_CSTP * salinidade * temperatura * pressao + coef_CS2TP * pow(salinidade,2) * temperatura * pressao \
                       + coef_CS2P2 * pow(salinidade,2) * pow(pressao,2)
        #
        # c(S,T,P) = C000 + ΔCT + ΔCS + ΔCP + ΔCSTP
        resultado_equacao = coef_C000 + equacao_CT + equacao_CS + equacao_CSTP
        #
        return resultado_equacao

def equacao_NPL(self, temperatura, salinidade, profundidade):
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
                'Informe uma profundidade válidaentre 0 a 8000 metros')
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


# if __name__ =="__main__":
#     calcula = CalcularVelocidade()
#     temperatura =
#     salinidade =
#     pressao =
#     calcula.equacao_mackenzie(temperatura, salinidade, pressao)
