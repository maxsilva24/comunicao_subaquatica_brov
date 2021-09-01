from logging import exception
import unittest
from BROV_Comunicacao  import CalcularVelocidadeSomAgua, TipoEquacao

class Test_Comunicacao(unittest.TestCase):
    
    def setUp(self):
        self.calcula_velocidade_som = CalcularVelocidadeSomAgua()
    
    def test_equacao_mackenzie(self):
        resutado_esperado = 1501.3503029378637
        temperatura = 17
        salinidade = 25
        profundidade = 10
        resultado= self.calcula_velocidade_som.calcular_equacao_mackenzie(temperatura, salinidade, profundidade)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')

    def test_equacao_mackenzie_validacao(self):
        temperatura = 17  #valido 2 a 30 graus Celsius
        salinidade = 25   #valido 25 a 40 partes por mil
        profundidade = 10 #valido 0 a 8000 metros
        #temperatura
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_mackenzie, 1, salinidade, profundidade)
        # salinidade
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_mackenzie, temperatura, 2, profundidade)
        # profundidade 
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_mackenzie, temperatura, salinidade, 5000000)

    def test_equacao_coppens(self):
        resutado_esperado = 1474.0219993
        temperatura = 9
        salinidade = 25
        profundidade = 10
        resultado= self.calcula_velocidade_som.calcular_equacao_coppens(temperatura, salinidade, profundidade)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')    
    
    def test_equacao_coppens_validacao(self):
        temperatura = 17  #valido 0 a 35°C graus Celsius
        salinidade = 25   #valido 0 a 45 partes por mil
        profundidade = 10 #valido 0 a 4000 metros
        #temperatura
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_coppens, 45, salinidade, profundidade)
        # salinidade
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_coppens, temperatura, -10, profundidade)
        # profundidade 
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_coppens, temperatura, salinidade, 5000000)    

    def test_equacao_unesco(self):
        resutado_esperado = 1473.9654897172309
        temperatura = 9
        salinidade = 25
        pressao = 10
        resultado= self.calcula_velocidade_som.calcular_equacao_unesco(temperatura, salinidade, pressao)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')             

    def test_equacao_unesco_validacao(self):
        temperatura = 17  #valido 0 a 40°C graus Celsius
        salinidade = 25   #valido 0 a 40 partes por mil
        pressao = 10 #valido (0 a 100000 Kpa)
        #temperatura
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_unesco, 45, salinidade, pressao)
        # salinidade
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_unesco, temperatura, -10, pressao)
        # profundidade 
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_unesco, temperatura, salinidade, -5000000)  

    def test_equacao_del_grosso(self):
        resutado_esperado = 1482.4691698362064
        temperatura = 9
        salinidade = 32
        pressao = 10
        resultado= self.calcula_velocidade_som.calcular_equacao_del_grosso(temperatura, salinidade, pressao)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')   

    def test_equacao_del_grosso_validacao(self):
        temperatura = 17  #valido 0 a 30°C graus Celsius
        salinidade = 25   #valido 30 a 40 partes por mil
        pressao = 10 #valido (0 a 1000 kg/cm²) onde 100 kPa=1,019716 kg/cm2. (Wong e Zhu, 1995).
        #temperatura
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_del_grosso, 45, salinidade, pressao)
        # salinidade
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_del_grosso, temperatura, -10, pressao)
        # profundidade 
        self.assertRaises(Exception,                          
                          self.calcula_velocidade_som.calcular_equacao_del_grosso, temperatura, salinidade, -5000000)     

    def test_calcular_velocidade_som_mackenzie(self):        
        resutado_esperado = 1501.3503029378637
        temperatura = 17
        salinidade = 25
        profundidade = 10
        resultado= self.calcula_velocidade_som.calcular_velocidade_som(TipoEquacao.MACKENZIE, temperatura=temperatura, 
                                                        salinidade=salinidade, profundidade=profundidade)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')
        #
    def test_calcular_velocidade_som_coppens(self):        
        resutado_esperado = 1474.0219993
        temperatura = 9
        salinidade = 25
        profundidade = 10
        resultado= self.calcula_velocidade_som.calcular_velocidade_som(TipoEquacao.COPPENS, temperatura=temperatura, 
                                                        salinidade=salinidade, profundidade=profundidade)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')
        #
    def test_calcular_velocidade_som_unesco(self):          
        resutado_esperado = 1473.9654897172309
        temperatura = 9
        salinidade = 25
        pressao = 10
        resultado= self.calcula_velocidade_som.calcular_velocidade_som(TipoEquacao.UNESCO,temperatura=temperatura, 
                                                        salinidade=salinidade, pressao=pressao)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}') 
        #
    def test_calcular_velocidade_som_del_grosso(self):  
        resutado_esperado = 1482.4691698362064
        temperatura = 9
        salinidade = 32
        pressao = 10
        resultado= self.calcula_velocidade_som.calcular_velocidade_som(TipoEquacao.DEL_GROSSO,temperatura=temperatura, 
                                                        salinidade=salinidade, pressao=pressao)
        self.assertEqual(resultado, resutado_esperado, 
                         msg=f'Resultado {resultado} não é igual ao resultado esperado {resutado_esperado}')                        

       
