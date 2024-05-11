from abaqus import *
from abaqusConstants import *
import random
import regionToolset
#create model
myModel=mdb.Model(name='X')
del mdb.models['Model-1']
l_a=12  #Basewidth
l_b=22 #Baselength
l_c=20  #Basehigh
# information of CRACK
a=50e-3 #a
b=1.5e-3#b
li=10e-3#c
N=1500#N
#define defect scope
length=22-0.3 #
width=l_a-0.3  #Y
height=l_c-0.3#
#create base
#1)create sketch
mySketch=myModel.ConstrainedSketch(name='Panel',sheetSize=100)
mySketch.rectangle(point1=(-l_b/2+length/2,0),point2=(l_b/2+length/2,l_a))
#2)create part
myPart1=myModel.Part(name='panel', dimensionality=THREE_D,type=DEFORMABLE_BODY)
myPart1.BaseSolidExtrude(sketch=mySketch,depth=l_c)


#create material
myMaterial = myModel.Material(name='Q235')
myMaterial.Density(table=((7.8e-9,),))
myMaterial.Elastic(table=((206e3,0.29),))
#creat section
myModel.HomogeneousSolidSection(name='Section-1', material='Q235', thickness=None)
#Assigned section
my_region = (myPart1.cells,)
myPart1.SectionAssignment(region=my_region, sectionName='Section-1')

#step
myModel.ExplicitDynamicsStep(name='Step-1', previous='Initial', timePeriod=7e-06)

# define interact of 
def interact_judgement(points, line, diameter):
    c = line[0]
    d = line[1]
    #
    dis = []
    num = 50
    sign = True
    for point in points:
        a = point[0]
        b = point[1]
        for i in range(num+1):
            mx = c[0] + (d[0] - c[0]) * i * (1. / num)
            my = c[1] + (d[1] - c[1]) * i * (1. / num)
            mz = c[2] + (d[2] - c[2]) * i * (1. / num)
            for j in range(num+1):
                nx = a[0] + (b[0] - a[0]) * j * (1. / num)
                ny = a[1] + (b[1] - a[1]) * j * (1. / num)
                nz = a[2] + (b[2] - a[2]) * j * (1. / num)
                # distance calculate
                distance = sqrt((mx-nx)**2+(my-ny)**2+(mz-nz)**2)
                dis.append(distance)
                if distance <= diameter:
                    sign = False
                    break
            if not sign:
                break
        if not sign:
            break
    # if sign:
    #     print(min(dis),max(dis))
    return sign


#create crack
#create sketch
mySketch=myModel.ConstrainedSketch(name='crack',sheetSize=100)
mySketch.EllipseByCenterPerimeter(center=(0.0, 0.0), axisPoint1=(a, 0.0), axisPoint2=(0.0, b))
#create part
myPart2=myModel.Part(name='crack', dimensionality=THREE_D,type=DEFORMABLE_BODY)
myPart2.BaseSolidExtrude(sketch=mySketch,depth=li)

# save trans and rotate information
fibre = []
points = []
# caculate the movement and rotation of fibre
# interact of judgement
for num in range(N):
    x = random.uniform(4, 18)
    y = random.uniform(0.3, width)
    z = random.uniform(3, 17)
    angle_y = random.uniform(0, 360)
    angle_z = random.uniform(0, 360)
    z2 = z + li*cos(radians(angle_y))
    x2 = x + li*sin(radians(angle_y))*cos(radians(angle_z))
    y2 = y + li*sin(radians(angle_y))*sin(radians(angle_z))
    point = ((x,y,z), (x2,y2,z2))
    if len(points) == 0:
        points.append(point)
        fibre.append([x, y, z, angle_y, angle_z])
    elif interact_judgement(points, point, 2*a):
        points.append(point)
        fibre.append([x, y, z, angle_y, angle_z])
    else:
        pass

# create in Abaqus
a = myModel.rootAssembly
for num in range(len(fibre)):
    x = fibre[num][0]
    y = fibre[num][1]
    z = fibre[num][2]
    angle_y = fibre[num][3]
    angle_z = fibre[num][4]
    a.Instance(name='crack-{}'.format(num), part=myPart2, dependent=ON)
    a.rotate(instanceList=('crack-{}'.format(num),), axisPoint=(0, 0, 0), axisDirection=(0, 0, 1), angle = angle_z)
    a.rotate(instanceList=('crack-{}'.format(num),), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0), angle = angle_y)
    a.translate(instanceList=('crack-{}'.format(num), ), vector=(x, y, z))

# Merge assembly to Part
# Gets the names of all instances
all_instance_names = myModel.rootAssembly.instances.keys()
# Merge all instances into one instance
merged_instance_name = 'crack-all'
myModel.rootAssembly.InstanceFromBooleanMerge(name=merged_instance_name, instances=[myModel.rootAssembly.instances[name] for name in all_instance_names], originalInstances=DELETE, keepIntersections=ON)

