# Comunicao Subaquática Brov
Este projeto trata arquivos de apoio de simulação de comunicação subaquática para o BROV, o node que calcula o delay foi feito no ROS1 e ROS2

Todas as instruções abaixo, entende-se que o ROS2 esta instalado com o python 3 e o ROS14 com O python 2.7 ( no linux)

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

## Passos para Rodar o ROS1
   * Navegue até a pasta raiz **../ROS1_Node/brov_ws**
   * Compile o projeto na pasta raiz
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ catkin_make
~~~ 
   * Transforme o arquivo em no_comunicacao_brov.py em executavél
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ chmod +x ../no_comunicacao_brov.py
~~~ 
   * Instale o contexto do ros no terminal atual executando o arquivo setup dentro da pasta devel 
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ source devel/setup.bash
~~~   
   * Execute o nó master do ros
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ roscore
~~~   
   * Execute o node *no_comunicacao_brov.py* do pacote *comunicacao_brov* passando as coodernadas X,Y e Z de origem e destino para calcular o delay
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ rosrun comunicacao_brov no_comunicacao_brov.py 2.0 3.0 4.0 5.0 6.0 7.0
~~~

##Executar o AUVNETSim
* Navegue até a pasta raiz **../ROS1_Node**
* Instalar o virtual env no python 2
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ python2.7 -m pip install virtualenv
~~~
* Criar ambiente virtual
~~~console
   #Por exemplo.: 
   max@max-HP-ProBook-6460b:../ROS1_Node$ python2.7 -m virtualenv v_env
~~~
* Ativar a maquina virtual
~~~console
   #Por exemplo.: 
   (v_env) max@max-HP-ProBook-6460b:../ROS1_Node$ source v_env/bin/activate
~~~
* Executar o arquivo teste_auv_net_sim.py
~~~console
   #Por exemplo.: 
   (v_env) max@max-HP-ProBook-6460b:../ROS1_Node$ python src/comunicacao_brov/scripts/teste_auv_net_sim.py
~~~