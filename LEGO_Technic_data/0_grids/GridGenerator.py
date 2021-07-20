# -*- coding: utf-8 -*-
# @Time    : 26/12/2018 11:41 AM
# @Author  : LI RUI HUI
# @Description:
# @Email   : ruihuili.lee@gmail.com
# @File    : GridGenerator.py

#Basic parameter
depth = 0
scale=4


def write_obj(vertexs,lines,width):
    with open('grid_{}.obj'.format(width), 'w') as f:
        f.write('#List of vertexs \n')
        for i in range(len(vertexs)):
            f.write("v "+str(vertexs[i][0])+" "+str(vertexs[i][1])+" "+str(vertexs[i][2])+"\n")
        f.write('#List of lines \n')
        for i in range(len(lines)):
            f.write("l " + str(lines[i][0]+1) + " " + str(lines[i][1]+1) + "\n")


def generate_grid(vertex_num_x,vertex_num_y):
    vertexs = []
    lines = []
    for i in range(vertex_num_y):
        for j in range(vertex_num_x):
            vertexs.append([i*scale,j*scale,depth])
    for i in range(vertex_num_y):
        for j in range(vertex_num_x):
           if j+1 < vertex_num_x:
               lines.append([i*vertex_num_x+j,i*vertex_num_x+j+1])
           if i+1 < vertex_num_y:
               lines.append([i*vertex_num_x+j,(i+1)*vertex_num_x+j])
    return vertexs,lines


width = 100
height = 5
for i in range(1,width+1):
    vertexs,lines = generate_grid(i+1,i+1)
    write_obj(vertexs,lines,i)
    print('Done!')







