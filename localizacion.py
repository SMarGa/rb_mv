#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Rob�tica Computacional 
# Grado en Ingenier�a Inform�tica (Cuarto)
# Pr�ctica 5:
#     Simulaci�n de robots m�viles holon�micos y no holon�micos.

#localizacion.py

import math
import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
# ******************************************************************************
# Declaraci�n de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def mostrar(objetivos,ideal,trayectoria):
  # Mostrar objetivos y trayectoria:
  #plt.ion() # modo interactivo
  # Fijar los bordes del gr�fico
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  ideT   = np.array(ideal).T.tolist()
  bordes = [min(trayT[0]+objT[0]+ideT[0]),max(trayT[0]+objT[0]+ideT[0]),
            min(trayT[1]+objT[1]+ideT[1]),max(trayT[1]+objT[1]+ideT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar objetivos y trayectoria
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  r = radio * .1
  for p in trayectoria:
    plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r')
    #plt.plot(p[0],p[1],'or')
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.o')
  plt.show()
  input()
  plt.clf()

def localizacion(balizas, real, ideal, centro, radio, mostrar=0):
  # Buscar la localizaci�n m�s probable del robot, a partir de su sistema
  # sensorial, dentro de una regi�n cuadrada de centro "centro" y lado "2*radio".

  # Crear lista imagen, llamando a measurement_prob en cada una de las celdas
  imagen = []
  best_prop = 10000.
  best_move = [0,0]
  count = 0

  
  for i in np.arange(-radio,radio, 0.1):
    imagen.append([])
    for j in np.arange(-radio,radio, 0.1):
      ideal.set(centro[0] + j, centro[1] + i, ideal.orientation)
      prop = ideal.measurement_prob(real.sense(balizas),balizas)
      imagen[count].append(prop)
      if(best_prop > prop):
        best_prop = prop
        best_move[0] = centro[0] + j
        best_move[1] = centro[1] + i
    count += 1

  ideal.set(best_move[0], best_move[1], ideal.orientation)

  best_angle = ideal.orientation
  best_prop = 10000.
  for i in np.arange(-pi,pi,0.1):
    ideal.set(ideal.x, ideal.y, i)
    prop = ideal.measurement_prob(real.sense(balizas),balizas)
    if(best_prop > prop):
      best_angle = i
      best_prop = prop

  ideal.set(ideal.x, ideal.y, best_angle)


  if mostrar:
    #plt.ion() # modo interactivo
    plt.xlim(centro[0]-radio,centro[0]+radio)
    plt.ylim(centro[1]-radio,centro[1]+radio)
    imagen.reverse()
    plt.imshow(imagen,extent=[centro[0]-radio,centro[0]+radio,\
                              centro[1]-radio,centro[1]+radio])
    balT = np.array(balizas).T.tolist();
    plt.plot(balT[0],balT[1],'or',ms=10)
    plt.plot(ideal.x,ideal.y,'D',c='#ff00ff',ms=10,mew=2)
    plt.plot(real.x, real.y, 'D',c='#00ff00',ms=10,mew=2)
    plt.show()
    input()
    plt.clf()

# ******************************************************************************

# Definici�n del robot:
P_INICIAL = [0.,4.,0.] # Pose inicial (posici�n y orientacion)
V_LINEAL  = .7         # Velocidad lineal    (m/s)
V_ANGULAR = 140.       # Velocidad angular   (�/s)
FPS       = 10.        # Resoluci�n temporal (fps)

HOLONOMICO = 1
GIROPARADO = 0
LONGITUD   = .2

# Definici�n de trayectorias:
trayectorias = [
    [[1,3]],
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)]
    ]

# Definici�n de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <indice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definici�n de constantes:
EPSILON = .1                # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma

ideal = robot()
ideal.set_noise(0,0,.1)   # Ruido lineal / radial / de sensado
ideal.set(*P_INICIAL)     # operador 'splat'

real = robot()
real.set_noise(.01,.01,.1)  # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

random.seed(0)
tray_ideal = [ideal.pose()]  # Trayectoria percibida
tray_real = [real.pose()]     # Trayectoria seguida

tiempo  = 0.
espacio = 0.
#random.seed(0)
random.seed(datetime.now())

# Llamada a localización buscando en toda el grid y con mostrar = 1
max_len = 0
centro_med = [0,0]
for i in objetivos:
  if i[0] > max_len:
    max_len = i[0]
  if i[1] > max_len:
    max_len = i[1]
  centro_med[0] += i[0]
  centro_med[1] += i[1]

max_len = int(max_len)
centro_med[0] = centro_med[0] / len(objetivos)
centro_med[0] = int(centro_med[0])
centro_med[1] = centro_med[1] / len(objetivos)
centro_med[1] = int(centro_med[1])

localizacion(objetivos,real,ideal,centro_med,max_len*1.5,1)

for punto in objetivos:
  while distancia(tray_ideal[-1],punto) > EPSILON and len(tray_ideal) <= 1000:
    pose = ideal.pose()

    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0

    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:
        v = 0
      ideal.move(w,v)
      real.move(w,v)
    else:
      ideal.move_triciclo(w,v,LONGITUD)
      real.move_triciclo(w,v,LONGITUD)

    pose_ideal = ideal.pose()
    pose_real = real.pose()

    tray_ideal.append(pose_ideal)
    tray_real.append(pose_real)
    
    # Llamar localización si estan muy separados (umbral con measurement_prob)  con mostrar a 0 

  
    
    center_x = int((pose_ideal[0] + pose_real[0] )/ 2)
    center_y = int((pose_ideal[1] + pose_real[1] )/ 2)
    center_avg = [center_x,center_y]
    diff_puntos = math.sqrt((pose_ideal[0] - pose_real[0])**2 + (pose_ideal[1] - pose_real[1])**2)
    if(diff_puntos > 0.15):
      localizacion(objetivos,real,ideal,center_avg,1,0)
    
    espacio += v
    tiempo  += 1

if len(tray_ideal) > 1000:
  print ("<!> Trayectoria muy larga - puede que no se haya alcanzado la posicion final.")
print ("Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s")
print ("Distancia real al objetivo: "+\
    str(round(distancia(tray_real[-1],objetivos[-1]),3))+"m")
mostrar(objetivos,tray_ideal,tray_real)  # Representaci�n gr�fica

