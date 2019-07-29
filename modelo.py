import numpy as np
import math
import matplotlib.pyplot as plt
import random

#Definição das variáveis
#Primeiramente, devemos resolver a EDO
#O modelo é dado por Tf = (I + mr² + mlr.cos(phi_tl).phi_grp'' + (I + mr²)phi_tl'' + mlr.sin(phi_tl).phi_grp''² + mgr.cos(phi_grp + phi_tl))

#Definindo os steps e o tempo
steps = 1000
range_max = 2
time = np.linspace(0, range_max, num = steps)
dt = range_max/steps

#Função da aceleração do grip de entrada.
A = 1 #Amplitude máxima de phi_grp_dd
t = 0 #Tempo inicial 
w = 3 #Frequência do sinal


#Agora, vamos definir as variáveis a serem utilizadas
m = 0.500 #Massa
l = 0.35 #Comprimento
r = 0.1 #Distância da intersecção ao meio da ferramenta
I = 0.0001111 #Momento de inércia
g = 0 #Gravidade

#phi_grp_d = [0]
#phi_grp = [0]
#
#phi_tl_dd = [0] #Aceleração da ferramenta
#phi_tl_d = [0] #Velocidade da ferramenta
#phi_tl = [0]  #Angulação da ferramenta

mi_v = 0.066 #Coef de atrito
kmi_c = 9.906 #Coef de atrito
d0 = 0.0162 #Comprimento da ferramenta
dfing = 0.0161 #Comprimento dos dedos
fn = kmi_c*(d0 - dfing) #Força normal
tf = 0 #Torsional Friction

#Feito isso, vamos definir o modelo
class Modelo:
        
    def modelo(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, n):
        #Primeiraito mente, é feito o cálculo de tf
        tf =  ( mi_v*phi_tl_d[n-1] + fn*(np.sign(phi_tl_d[n-1])))*(-1)
        phi_tool_dd = ( tf - I - m*r**2 - m*l*r*math.cos( phi_tl[n-1] )*phi_grp_dd[n-1] - m*l*r*math.sin( phi_tl[n-1] )*phi_grp_d[n-1]*phi_grp_d[n-1] - m*r*g*math.cos( phi_grp[n-1] + phi_tl[n-1] ) )/(I + m*r*r)
        return phi_tool_dd
    
    def trajetoria(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, phi_max, tmax): #Realiza a modelagem da trajetória do robô. Nesse caso, usaremos um spline cúbico.
       
        phi_gr = ( (-2)*phi_max*math.pow(t, 3) )/math.pow(tmax, 3) + ( 3*phi_max*math.pow(t, 2) )/math.pow(tmax, 2) #O spline cúbico é uma função de 3 grau com as devidas condições de contorno  
        phi_gr_d = ( (-6)*phi_max*math.pow(t, 2) )/math.pow(tmax, 3) + ( 6*phi_max*math.pow(t, 1) )/math.pow(tmax, 2) #Derivando o espaço, temos a velocidade do braço
        phi_gr_dd = ( (-12)*phi_max*math.pow(t, 1) )/math.pow(tmax, 3) + ( 6*phi_max*math.pow(t, 0) )/math.pow(tmax, 2) #Derivando a velocidade, temos a aceleração.
        
        return phi_gr, phi_gr_d, phi_gr_dd

    ##Printando e Plotando resultados
    def plota_resultados(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, grandeza):  #0 se quiser ver distância, 1 se quiser ver velocidade e 2 se quiser ver aceleração
       
        if(grandeza == 0): #Plota a distância
            plt.plot(time, phi_grp, color='blue')
            plt.plot(time, phi_tl, color='red')
            plt.title('Entrada(azul) e Saída(vermelho)')
            plt.xlabel('tempo')
            plt.ylabel('dist')
            plt.show()
    
            plt.plot(time, phi_grp)
            plt.title('Entrada')
            plt.xlabel('tempo')
            plt.ylabel('dist')
            plt.show()
            
            plt.plot(time, phi_tl,color = 'red')
            plt.title('Saída')
            plt.xlabel('tempo')
            plt.ylabel('dist')
            #plt.ylim(6.5,9.5)
            plt.show()
            
        elif(grandeza == 1): #Plota a velocidade
            plt.plot(time, phi_grp_d, color='blue')
            plt.plot(time, phi_tl_d, color='red')
            plt.title('Entrada(azul) e Saída(vermelho)')
            plt.xlabel('tempo')
            plt.ylabel('velocidade')
            plt.show()
    
            plt.plot(time, phi_grp_d)
            plt.title('Entrada')
            plt.xlabel('tempo')
            plt.ylabel('velocidade')
            plt.show()
            
            plt.plot(time, phi_tl_d, color = 'red')
            plt.title('Saída')
            plt.xlabel('tempo')
            plt.ylabel('velocidade')
            #plt.ylim(6.5,9.5)
            plt.show()
            
        elif(grandeza == 2): #Plota a aceleração
            plt.plot(time, phi_grp_dd, color='blue')
            plt.plot(time, phi_tl_dd, color='red')
            plt.title('Entrada(azul) e Saída(vermelho)')
            plt.xlabel('tempo')
            plt.ylabel('aceleração')
            plt.show()
    
            plt.plot(time, phi_grp_dd)
            plt.title('Entrada')
            plt.xlabel('tempo')
            plt.ylabel('aceleração')
            plt.show()
            
            plt.plot(time, phi_tl_dd, color = 'red')
            plt.title('Saída')
            plt.xlabel('tempo')
            plt.ylabel('aceleração')
            #plt.ylim(6.5,9.5)
            plt.show()
    

          