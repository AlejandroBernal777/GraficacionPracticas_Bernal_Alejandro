#Importación de las librerias usadas en el program
import os
import cv2
import imutils
import numpy as np
import tkinter as tk
from turtle import *
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GL.shaders import *
from PIL import Image
import pygame
from pygame.locals import *
from vectors import *
from math import *
import sys
from utils.vector import Vector3
from utils.camera import Camera
from utils.light import Light
from utils.mesh.base import Mesh
from utils.mesh.meshes import *
from utils.mesh.spheres import *
from utils.mesh.point import *
from utils.matrix import *
from utils.tools import *
from utils.world import Scene


#------------------------------------------------------------------------#
#Funcion que nos lleva al paint de figuras
def pizarraOpenCV():
    
    clicks = []
    alto = 750
    ancho = 750
    
    def visualizarPuntos(event):
        clicks.append((event.x - 1, event.y - 1))
        canvas.create_oval(event.x-2, event.y-2, event.x+2, event.y+2, fill="")
        point = (event.x-ancho//2, event.y-alto//2)
        canvas.create_text(event.x-4, event.y-10, fill="black", font="Times 10", text="("+str(point[0])+", "+str(point[1])+")")

    def trazarLinea(event):
        canvas.unbind("<Button-1>", trazar)
        canvas.unbind("<Button-3>", terminar)
        trazos = Trazo(canvas, clicks)
        trazos.generarTrazo()
    
    def traslacion():        
        lista = list(map(float,input("Ingrese el valor de X y Y al que desea trasladar la figuras ").split()))
        traslado = Trazo(canvas, clicks)
        traslado.traslacionFigura(lista[0], lista[1])
        print("Que otra operacion desea hacer?")

    def rotacion():
        rot = Trazo(canvas, clicks)
        rot.generarTrazo()
        angulo = float(input("Ingrese el angulo de rotacion: "))
        angulo = angulo * np.pi/180
        rot.rotarFigura(angulo, alto/2, ancho/2)
        print("Que otra operacion desea hacer?")

    def escalado():
        lista = list(map(float,input("Ingrese el valor de X y Y al que desea scalar la figura ").split()))
        escl = Trazo(canvas, clicks)
        escl.generarTrazo()
        escl.escaladoFigura(lista[0], lista[1], alto/2, ancho/2)
        print("Que otra operacion desea hacer?")

    class Trazo(tk.Canvas):

        def __init__(self, canvas, vertices):
            self.vertices = vertices
            self.canvas = canvas

        def generarTrazo(self):
            return self.canvas.create_polygon(self.vertices, outline = "black", fill="red")
                
        def agregarvertex(self,vertex):
            self.vertices.append(vertex)

        def agregarvertices(self,vertices):
            self.vertices.append(vertices)

        def obtenervertices(self):
            return self.vertices

        def traslacionFigura(self, tx, ty):

            traslate =  np.asarray([[1, 0, tx], [0, 1, ty], [0, 0, 1]])
            lista = []
            for trs in self.vertices:
                lista.append([trs[0], trs[1], 1])
                
            nvertices = np.transpose(np.asarray(lista))
            matrizSalida = np.transpose(traslate.dot(nvertices))

            lista = []
            for trs in matrizSalida:
                lista.append((trs[0], trs[1]))
                
            self.vertices = lista
            self.generarTrazo()

        def rotarFigura(self, zeta, Dx, Dy):

            print(zeta, Dx, Dy)
            Dx = self.vertices[0][0]
            Dy = self.vertices[0][1]

            rotacion = np.asarray([[np.cos(zeta), - np.sin(zeta), Dx * (1-np.cos(zeta)) + Dy * np.sin(zeta)], [np.sin(zeta), np.cos(zeta), Dy * (1-np.cos(zeta)) - Dx*np.sin(zeta)], [0, 0, 1]])

            lista = []

            for fmatriz in self.vertices:
                lista.append([fmatriz[0], fmatriz[1], 1])
                
            print(self.vertices)
            nvertices = np.transpose(np.asarray(lista))
            matrizSalida = np.transpose(rotacion.dot(nvertices))

            lista = []
            for fmatriz in matrizSalida:
                lista.append((fmatriz[0], fmatriz[1]))

            self.vertices = lista
            print(self.vertices)
            self.generarTrazo()

        def escaladoFigura(self, Dx, Dy, xr, yr):

            xr = self.vertices[0][0]
            yr = self.vertices[0][1]

            escalar = np.asarray([[Dx, 0, xr * (1-Dx)], [0, Dy, yr * (1-Dy)], [0, 0, 1]])
            lista = []

            for fmatriz in self.vertices:
                lista.append([fmatriz[0], fmatriz[1],1])

            print(self.vertices)
            nvertices = np.transpose(np.asarray(lista))
            matrizSalida = np.transpose(escalar.dot(nvertices))

            lista = []
            for fmatriz in matrizSalida:
                lista.append((fmatriz[0], fmatriz[1]))
            
            self.vertices = lista
            print(self.vertices)
            self.generarTrazo()

    pizarra = Tk()
    pizarra.title("PIZARRA DE FIGURAS")
    pizarra.resizable(1,1)

    contenedor = Frame(pizarra, width=ancho, height=alto)
    contenedor.pack(side="top", fill="both", expand=True)
    contenedor.grid_rowconfigure(0, weight=1)
    contenedor.grid_columnconfigure(0, weight=1)

    ltext = Label(contenedor, text="PIZARRA DE FIGURAS CON TRASNFORMACIONES")
    ltext.pack(pady=10, padx=10)
    b1 = Button(contenedor, text="Traslacion", bg="#6897B8", fg="white", command=traslacion)
    b1.pack(pady=2)    
    b2 = Button(contenedor, text="Rotacion", bg="#E9288E", fg="white", command=rotacion)
    b2.pack(pady=2) 
    b3 = Button(contenedor, text="Escalado", bg="#AD1818", fg="white", command=escalado)
    b3.pack(pady=2) 

    canvas = Canvas(contenedor, height=alto, width=ancho, bg="#A9DDD6")
    """canvas.create_line(0, alto/2, ancho, alto/2)
    canvas.create_line(ancho/2, 0, ancho/2, alto)
    for i in range(0, ancho//2, 50):
        #X Positivas
        canvas.create_line(i+ancho//2, alto//2, i+ancho//2, alto//2+3)
        canvas.create_text(i+ancho//2, alto/2+10, fill="black", font="Times 10", text=str((i)))
        #X Negativas
        canvas.create_line(i, alto//2, i, alto//2+3)    
        canvas.create_text(i, alto/2+10, fill="black", font="Times 10", text=str(-(ancho//2-i)))
    for i in range(50, alto//2, 50):
        #Y Positivas
        canvas.create_line(ancho//2, i, ancho//2-5, i)
        canvas.create_text(ancho//2-17, i, fill="black", font="Times 10", text=str((alto//2 - i)))
        #Y Negativas
        canvas.create_line(ancho//2, i+alto//2, ancho//2-5, i+alto//2)
        canvas.create_text(ancho//2-17, i+alto//2, fill="black", font="Times 10", text=str(-(alto//2-i)))"""

    trazar = canvas.bind("<Button-1>", visualizarPuntos)
    terminar = canvas.bind("<Button-3>", trazarLinea)

    canvas.pack()

    pizarra.mainloop()

#------------------------------------------------------------------------#
#Funcion que nos lleva al reconocimiento de objetos
def cameraOpenCV():

    def entrenarObj():

        btn.configure(state="active")

        objetos = 'p'
        if not os.path.exists(objetos):
            messagebox.showinfo(message="CARPETA DE IMAGENES POSITIVAS CREADA", title="POSITIVAS")
            os.makedirs(objetos)

        cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)

        x1, y1 = 190, 80
        x2, y2 = 450, 398

        contador = 0

        while True:
            ret, frame = cap.read()
            if ret == False:
                break
            imAux = frame.copy()
            cv2.rectangle(frame,(x1,y1),(x2,y2),(255,0,0),2)

            obj = imAux[y1:y2,x1:x2]
            obj = imutils.resize(obj, width=38)

            k = cv2.waitKey(1)
            if k == 27:
                break

            if k == ord('s'):
                cv2.imwrite(objetos+'/objeto_{}.jpg'.format(contador), obj)
                print('Imagen almacenada: ', 'ob_{}.jpg'.format(contador))
                contador = contador + 1

            cv2.imshow('frame', frame)
            cv2.imshow('objeto', obj)

        cap.release()
        cv2.destroyAllWindows()

    def reconocerObj():

        btn.configure(state="active")

        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

        objClassif = cv2.CascadeClassifier('cascade.xml')

        while True:

            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            toy = objClassif.detectMultiScale(gray, scaleFactor = 5, minNeighbors = 91, minSize =(70,78))

            for (x,y,w,h) in toy:
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, 'Objeto', (x,y-10), 2, 0.7, (0,255,0), 2, cv2.LINE_AA)

            cv2.imshow('OBJETO',frame)

            if cv2.waitKey(1) == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

    ventanaCamera = Tk()
    linfo = Label(ventanaCamera, text="RECONOCIMIENTO DE OBJETOS", font="bold")
    linfo.grid(column=0, row=0, columnspan=2)

    selected = IntVar() 
    btn1 =  Button(ventanaCamera, text="Entrenar objeto", width=20, command=entrenarObj)
    btn2 = Button(ventanaCamera, text="Reconocer objeto", width=20, command=reconocerObj)

    selected.set(0)
    
    btn1.grid(column=0, row=1)
    btn2.grid(column=1, row=1)

    lcamera = Label(ventanaCamera, text="", width=20)
    lcamera.grid(column=0, row=2)

    lcameravideo = Label(ventanaCamera)
    lcameravideo.grid(column=0, row=3, columnspan=2)

    btn = Button(ventanaCamera, text="TERMINAR", state="disabled")
    btn.grid(column=0, row=4, columnspan=2, pady=10)

    ventanaCamera.mainloop()

#------------------------------------------------------------------------#
#Funcion que nos lleva a la figura 3D
def objetoOpenGL():
    Size = 1000, 1000
    Width, Height = Size
    BackgroundColor = (0 , 0, 0)
    aspect = Height/Width

    clipping = 1.5
    mouse_sensitivity = 200
    Zoffset = 7

    dim = 0.01

    #colors
    blue = (0, 0, 255)
    orange = (255, 160, 0)
    black = (0, 0, 0)
    white = (255, 255, 255)
    red = (255, 0, 0)
    green = (0, 255, 0)
    yellow = (0, 255, 255)

    def HandleEvent(camera, deltaTime):
        running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            if event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    x, y = event.rel
                    x /= mouse_sensitivity
                    y /= mouse_sensitivity
                    camera.HandleMouseEvent(x, y, deltaTime)

        return running

    pygame.init()

    flags = DOUBLEBUF
    screen = pygame.display.set_mode(Size, flags, 16)
    clock = pygame.time.Clock()
    fps = 60

    pygame.mouse.get_rel()
    pygame.mouse.set_visible(True)
    a = pygame.event.set_grab(False)

    sphere = Mesh()
    sphere.triangles = IcosphereTriangles((246,131,15), 2)
    sphere.position = Vector3(0, 0, 0)


    scene = Scene()
    scene.world.append(sphere)

    camera = Camera(Vector3(0, 0, 0), 0.1, 1000.0, 75.0)
    camera.speed = 0.5
    camera.rotationSpeed = 0.8
    
    light = Light(Vector3(0.9, 0.9, -1))
    hue = 0

    angle = 0
    moveLight = True
    run = True

    while run:
        screen.fill(BackgroundColor)
        clock.tick(fps)
        dt = clock.tick(fps)/100
        frameRate = clock.get_fps()
        pygame.display.set_caption(str(frameRate) + " fps")
        run = HandleEvent(camera, dt)
        hue = 0
        camera.HandleInput(dt)

        if moveLight == True and light != None:
            mx, my = pygame.mouse.get_pos()
            _x = translateValue( mx, 0,  Width,  -1,  1)
            _y = translateValue( my, 0, Height, -1, 1)
            light = Light(Vector3(-_x, -_y, -1))

        sphere.transform = Matrix.rotation_x(angle) @ (
            Matrix.rotation_y(angle) @ Matrix.scaling(1.4)
        )

        scene.update(
            dt = dt,
            camera=camera,
            light=light,
            screen=screen,
            showAxis=True,
            fill=True,
            wireframe=False,
            vertices=False,
            depth=True,
            clippingDebug=False,
            showNormals=False,
            radius=9,
            verticeColor=False,
            wireframeColor=(255, 255, 255),
            ChangingColor=hue)


        pygame.display.flip()
        angle += 0.01

    pygame.quit()
    sys.exit()


#------------------------------------------------------------------------#
#Creacion de la ventana del formulario principal
windowmain = Tk()
windowmain.title("PRACTICAS GRAFICACION OPENCV-OPENGL")
windowmain.geometry("450x200")
windowmain.config(bg="#3A3B3B")

lmenu = Label(text="BIENVENIDO A LAS PRACTICAS CON OPENCV Y OPENGL", bg="#3A3B3B", pady=5, font=('Epyptienne', 12, 'bold')).pack()
b1 = Button(text="Pizarra", bg="red", fg="white", font=('Comic Sans', 12), width=15, pady=5, height=1, command=pizarraOpenCV).pack()
b2 = Button(text="Camera", bg="red", fg="white", font=('Comic Sans', 12), width=15, pady=5, height=1, command=cameraOpenCV).pack()
b3 = Button(text="Figura 3D", bg="red", fg="white", font=('Comic Sans', 12), width=15, pady=5, height=1, command=objetoOpenGL).pack()

windowmain.mainloop()