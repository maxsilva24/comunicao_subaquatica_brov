from logging import exception
import unittest
from BROV_Comunicacao  import CalcularVelocidadeSomAgua

class Test_Comunicacao(unittest.TestCase):
    def test_equacao_mackenzie(self):
        calcula = CalcularVelocidadeSomAgua()
        resutado_esperado = 1501.3503029378637
        temperatura = 17
        salinidade = 25
        profundidade = 10
        resultado= calcula.equacao_mackenzie(temperatura, salinidade, profundidade)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado{resutado_esperado}')

    def test_equacao_mackenzie_validacao(self):
        calcula = CalcularVelocidadeSomAgua()
        temperatura = 17  #valido 2 a 30 graus Celsius
        salinidade = 25   #valido 25 a 40 partes por mil
        profundidade = 10 #valido 0 a 8000 metros
        #temperatura
        self.assertRaises(Exception,                          
                          calcula.equacao_mackenzie, 1, salinidade, profundidade)
        # salinidade
        self.assertRaises(Exception,                          
                          calcula.equacao_mackenzie, temperatura, 2, profundidade)
        # profundidade 
        self.assertRaises(Exception,                          
                          calcula.equacao_mackenzie, temperatura, salinidade, 5000000)
       
