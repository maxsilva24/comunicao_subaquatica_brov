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
            A equação acima para a velocidade do som na água do mar em função da temperatura, 
            salinidade e profundidade é dada pela equação de Mackenzie (1981).
            Faixa de validade: temperatura 2 a 30°C, salinidade 25 a 40 partes por mil, profundidade 0 a 8000 m
        Args:
            temperatura (float): temperatura em graus Celsius (2 a 30°C)
            salinidade (float): salinidade em partes por mil (25 a 40)
            profundidade (float): profundidade em metros (0 a 8000m)
        Result
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
