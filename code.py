import numpy as np
import math
import modelo
from modelo import Modelo

#Definição dos vetores a serem utilizados
#phi_grp_dd = [A*math.sin(w*t)]
phi_grp_dd = [6*math.pi/modelo.range_max**2]
phi_grp_d = [0]
phi_grp = [0]

phi_tl_dd = [0] #Aceleração da ferramenta
phi_tl_d = [0] #Velocidade da ferramenta
phi_tl = [0]  #Angulação da ferramenta


#Agora, vamos fazer o Loop para calcular essa EDO

n = 0 #Contador de iterações
for modelo.t in modelo.time: 
    if(n > 0): #Caso não esteja na primeira iteração
        
        distancia, velocidade, aceleracao = Modelo.trajetoria(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, math.pi, tmax = modelo.range_max) #Função que modela a trajetória do braço do robô
        phi_grp.append(distancia)
        phi_grp_d.append(velocidade)
        phi_grp_dd.append(aceleracao)
            
        phi_tl_dd.append( Modelo.modelo(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, n) ) #Calcula o modelo e adiciona no vetor
        
        #Calculo das antiderivadas de phi_tl_dd e phi_grp_dd
        phi_tl_d.append( np.trapz(phi_tl_dd[n-1:n+1], dx = modelo.dt) ) #Calcula a integral de phi_tl_d em um intervalo dt
        phi_tl.append( np.trapz( phi_tl_d[n-1:n+1], dx = modelo.dt) )   #Calcula a integral de phi_tl em um intervalo dt
    
#        phi_grp_d.append( np.trapz(phi_grp_dd[n-1:n+1], dx = dt ) ) #Calcula a integral de phi_grp_d em um intervalo dt
#        phi_grp.append( np.trapz( phi_grp_d[n-1:n+1], dx = dt ) )   #Calcula a integral de phi_grp em um intervalo dt
    
    n = n + 1 #Avança n para a próxima iteração 
    

#Printando os resultados 
#0 para Espaço, 1 para Velocidade e 2 para Aceleração
Modelo.plota_resultados(phi_tl, phi_tl_d, phi_tl_dd, phi_grp, phi_grp_d, phi_grp_dd, 0) 
    