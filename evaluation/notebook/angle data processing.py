#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('matplotlib', 'inline')
import numpy as np
import matplotlib.pyplot as plt


# ### code example
# all_means=[]
# all_stds=[]
# all_step_means = []
# all_step_stds = []
# 
# for layer in range(1,6):
#     print("This is the result for Layer ",layer)
#     
#     stat = np.loadtxt(str(layer)+'.txt',skiprows=1)
#     means =[]
#     stds=[]
#     step_means=[]
#     step_stds=[]
# 
#     for j in range(5):    #0.25,0.5,0.75 and 200 data for each difficulties
#         cover = stat[100*j:100*(j+1),0]  # if plot the coverage use index=0, this is total_z axis movement
#         steps = stat[100*j:100*(j+1),4]
#         #cover = cover/steps
#         mean = np.mean(cover)
#         std = np.std(cover)
#         means.append(mean)
#         stds.append(std)
#         
#         step_means.append(np.mean(steps))
#         step_stds.append(np.std(steps))
#         
#     all_means.append(means)
#     all_stds.append(stds)
#     all_step_means.append(step_means)
#     all_step_stds.append(step_stds)
# 
#     print(means)
#     #print(stds)

# In[2]:


angle_data = np.loadtxt('angle.txt') #,skiprows=1)


# In[3]:


angle_data.shape


# In[4]:


angle_data = np.array(angle_data)


# In[5]:


print("there are how many sim_steps in 1 second:", 180000/ (30 *60))


# In[6]:


angle_data = angle_data[99:,:]
angle_data.shape


# In[7]:


angle_data[0],angle_data[4], angle_data[5]


# In[8]:


Second_interval = 1
interval = 100 * Second_interval
angle_data_interval = angle_data.reshape(-1, interval, 4)
angle_data_interval.shape


# In[10]:


# for every cluster calculate mean and std:
all_means=[]
all_stds=[]

for i in range(angle_data_interval.shape[0]):
    # means =[]
    # stds=[]
    
    angle_mean = np.mean(angle_data_interval, axis=2)
    # axis param is that the axis parameter enables you to calculate the mean of this specific 'axis' row or column.
    angle_std = np.std(angle_data_interval, axis=2)
    
    all_means.append(angle_mean)
    all_stds.append(angle_std)

all_means = np.array(all_means)
all_means.shape


# In[ ]:





# In[ ]:





# In[33]:


a = np.array([[1,2],[3,4],[5,6]])
a, np.mean(a, axis=0), np.mean(a, axis=1)


# In[ ]:





# In[ ]:





# In[ ]:





# In[10]:


temporal_data = angle_data[500:600, 1:]
temporal_data = temporal_data.T
temporal_data[0].shape


# In[11]:


for i in range(1):
    # set up a figure twice as wide as it is tall
    fig = plt.figure(figsize=(18, 10))
    
    
    temporal_data = angle_data[i*1000: (i+1)*1000-500, 1:]
    
    # fig.tight_layout()
    # fig = plt.figure(figsize=(8,6))
    # ax = plt.axes(projection='3d')
    
    # Data for a three-dimensional line
    # ax.plot3D(temporal_data[0], temporal_data[1], temporal_data[2], 'gray')
    # Data for three-dimensional scattered points
    
    #===============
    #  First subplot
    #===============
    # set up the axes for the first plot
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    temporal_data = temporal_data.T
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    
    # ax.set_zlim(-1.01, 1.01)
    # fig.colorbar(surf, shrink=0.5, aspect=10)
    
    
    temporal_data = angle_data[(i+1)*1000-500: (i+1)*1000, 1:]
    temporal_data = temporal_data.T
    
    #===============
    # Second subplot
    #===============
    # set up the axes for the second plot
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');

    # plt.show()


# In[ ]:





# In[9]:


for i in range(10):
    temporal_data = angle_data[i*1000: (i+1)*1000, 1:]
    fig = plt.figure(figsize=(8,6))
    ax = plt.axes(projection='3d')
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], 'gray')


# In[ ]:





# In[ ]:





# In[21]:


angle_data.shape[0]


# angle_data[179000:180000]

# In[23]:


angle_data[179000:180000].shape


# In[24]:


angle_data[178000:179000].shape


# In[ ]:





# In[27]:


import numpy as np
import matplotlib.pyplot as plt


x1 = np.linspace(0.0, 5.0)
x2 = np.linspace(0.0, 2.0)

y1 = np.cos(2 * np.pi * x1) * np.exp(-x1)
y2 = np.cos(2 * np.pi * x2)

plt.subplot(1, 2, 1)
plt.plot(x1, y1, 'ko-')
plt.title('A tale of 2 subplots')
plt.ylabel('Damped oscillation')


plt.subplot(1, 2, 2)
plt.plot(x2, y2, 'r.-')
plt.xlabel('time (s)')
plt.ylabel('Undamped')

plt.show()


# In[ ]:





# In[ ]:





# In[ ]:





# ## final code!!

# In[15]:


# a single plort is in the 5 second range
for i in range(180):
    fig = plt.figure(figsize=(18, 10))
    temporal_data = angle_data[i*1000: (i+1)*1000-500, 1:]
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    temporal_data = temporal_data.T
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    
    temporal_data = angle_data[(i+1)*1000-500: (i+1)*1000, 1:]
    temporal_data = temporal_data.T
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');


# In[17]:


# a single plort is in the N second range
N = 5
NN = N*100
import matplotlib.backends.backend_pdf
pdf = matplotlib.backends.backend_pdf.PdfPages("output.pdf")
for i in range(180):
    fig = plt.figure(figsize=(18, 10))
    temporal_data = angle_data[i* (NN*2): (i+1)*(NN*2)-NN, 1:]
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    temporal_data = temporal_data.T
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    
    temporal_data = angle_data[(i+1)*(NN*2)-NN: (i+1)*(NN*2), 1:]
    temporal_data = temporal_data.T
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    pdf.savefig( fig )
pdf.close()


# In[19]:


# a single plort is in the N second range
N = 2
NN = N*100
import matplotlib.backends.backend_pdf
pdf = matplotlib.backends.backend_pdf.PdfPages("output_2seconds.pdf")
for i in range(90*5):
    fig = plt.figure(figsize=(18, 10))
    temporal_data = angle_data[i* (NN*2): (i+1)*(NN*2)-NN, 1:]
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    temporal_data = temporal_data.T
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    
    temporal_data = angle_data[(i+1)*(NN*2)-NN: (i+1)*(NN*2), 1:]
    temporal_data = temporal_data.T
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    pdf.savefig( fig )
pdf.close()


# In[ ]:





# In[ ]:





# # Animated (Matplotlib) 3D Scatter Plot Trajectory

# In[22]:


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
 
# References
# https://gist.github.com/neale/e32b1f16a43bfdc0608f45a504df5a84
# https://towardsdatascience.com/animations-with-matplotlib-d96375c5442c
# https://riptutorial.com/matplotlib/example/23558/basic-animation-with-funcanimation
 
# ANIMATION FUNCTION
def func(num, dataSet, line):
    # NOTE: there is no .set_data() for 3 dim data...
    line.set_data(dataSet[0:2, :num])    
    line.set_3d_properties(dataSet[2, :num])    
    return line
 
 
# THE DATA POINTS
t = np.arange(0,20,0.2) # This would be the z-axis ('t' means time here)
x = np.cos(t)-1
y = 1/2*(np.cos(2*t)-1)
dataSet = np.array([x, y, t])
numDataPoints = len(t)
 
# GET SOME MATPLOTLIB OBJECTS
fig = plt.figure()
ax = Axes3D(fig)
 
# NOTE: Can't pass empty arrays into 3d version of plot()
line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='g')[0] # For line plot
 
# AXES PROPERTIES]
# ax.set_xlim3d([limit0, limit1])
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('time')
ax.set_title('Trajectory of electron for E vector along [120]')
 
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet,line), interval=50, blit=False)
#line_ani.save(r'AnimationNew.mp4')
 
 
plt.show()


# In[ ]:





# In[ ]:





# In[ ]:





# In[5]:


## smaller the y and x scale to (-0.6, 0.6)
#ax.set_xlim(-0.6,0.6)
#ax.set_ylim(-0.6,0.6)
# a single plort is in the N second range
N = 2
NN = N*100
import matplotlib.backends.backend_pdf
pdf = matplotlib.backends.backend_pdf.PdfPages("output_2seconds_smaller_scale.pdf")
for i in range(90*5):
    fig = plt.figure(figsize=(18, 10))
    temporal_data = angle_data[i* (NN*2): (i+1)*(NN*2)-NN, 1:]
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.set_xlim(-0.6,0.6)
    ax.set_ylim(-0.6,0.6)
    temporal_data = temporal_data.T
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    
    temporal_data = angle_data[(i+1)*(NN*2)-NN: (i+1)*(NN*2), 1:]
    temporal_data = temporal_data.T
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.set_xlim(-0.6,0.6)
    ax.set_ylim(-0.6,0.6)
    ax.scatter3D(temporal_data[0], temporal_data[1], temporal_data[2], cmap='Greens');
    pdf.savefig( fig )
pdf.close()


# In[ ]:




