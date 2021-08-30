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


# if __name__ =="__main__":
#     calcula = CalcularVelocidade()
#     temperatura =
#     salinidade =
#     pressao =
#     calcula.equacao_mackenzie(temperatura, salinidade, pressao)
