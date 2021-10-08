# Comunicao Subaquática Brov
Este projeto trata arquivos de apoio de simulação de comunicação subaquática para o BROV, o node que calcula o delay foi feito no ROS1 e ROS2
Todas as instruções abaixo, entende-se que o ROS2 esta instalado com o python 3 e o ROS14 com O python 2.7
Para instalar no **Linux** siga as instruções na pagina: https://docs.ros.org/.
Para instalar no **Windows** siga as instruções na pagina: https://ms-iot.github.io/ROSOnWindows/GettingStarted/Setup.html 
Instalar o ROS2 com  **docker** siga as instruções na pagina: https://www.allisonthackston.com/articles/vscode-docker-ros2.html 

## Passos para Rodar o ROS2
   * Navegue até a pasta raiz **../ROS2_Node**
   * Compile o projeto na pasta raiz
~~~console
   #Por exemplo.: 
   PS ../ROS2_Node> colcon build --merge-install
~~~
   * Instale o contexto do ros no terminal atual executando o arquivo setup dentro da pasta install 
~~~console
   #Por exemplo.: 
   PS ../ROS2_Node> \install\setup.ps1
~~~   
   * Execute o node *no_comunicacao_brov* do pacote *comunicacao_brov* passando as coodernadas X,Y e Z de origem e destino para calcular o delay
~~~console
   #Por exemplo.: 
   PS ../ROS2_Node> no_comunicacao_brov --ros-args -p coordenadada_origem:="[2.0,3.0,7.0]" -p coordenadada_destino:="[2.0, 1.5, 2.6]"
~~~ 
