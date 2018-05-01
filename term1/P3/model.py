#Import
import os
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import sklearn

from keras.models import Sequential,Model
from keras.layers import Flatten,Dense,Lambda,Cropping2D,Convolution2D,Dropout
from sklearn.model_selection import train_test_split
from random import shuffle

#generator function to read input files in batches
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        #Randomly shuffle input data
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            #Read batches
            batch_samples = samples[offset:offset+batch_size]
            images = [] #image data
            angles = [] #streering angles
            for batch_sample in batch_samples:
               #Read Center,Left and Right Images
               for i in range(3):
                  #File name
                  name = 'data/IMG/'+batch_sample[i].split('/')[-1]
                  image=cv2.imread(name)
                  images.append(image)
                  #If image is Center read the steering angle
                  if(i==0):
                    angle = float(batch_sample[3])
                  #If image is Left adjust steering angle by .25
                  elif(i==1):
                    angle = float(batch_sample[3])+.25
                  #If image is Right adjust steering angle by -.25
                  elif(i==2):
                    angle = float(batch_sample[3])-.25
                  angles.append(angle)
            #Augment the dataset by flipping the images and steering angles
            aug_images,aug_angles=[],[]
            for image,angle in zip(images,angles):
                aug_images.append(image)
                aug_angles.append(angle)
                aug_images.append(cv2.flip(image,1))
                aug_angles.append(angle*-1.0)
            #Create np array
            X_train = np.array(images)
            y_train = np.array(angles)
            #Shuffle and return
            yield sklearn.utils.shuffle(X_train, y_train)


#Read Training Data
samples = []
with open('data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)
    for line in reader:
        samples.append(line)

#shuffle input data
shuffle(samples)

#split input into training and validation set
train_samples, validation_samples = train_test_split(samples, test_size=0.2)


# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=128)
validation_generator = generator(validation_samples, batch_size=128)

#Image shape
ch, row, col = 3, 160, 320

#Neural network model
model=Sequential()
#Normalization
model.add(Lambda(lambda x: x/127.5 - 1.,input_shape=(row, col,ch),output_shape=(row, col,ch)))
#Cropping
model.add(Cropping2D(cropping=((70,25), (0,0)), input_shape=(row, col,ch)))
#Convolution Layers
model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
model.add(Dropout(0.25))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(Dropout(0.25))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
model.add(Dropout(0.25))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Dropout(0.25))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Dropout(0.25))
#Dense Layers
model.add(Flatten())
model.add(Dense(100))
model.add(Dropout(0.25))
model.add(Dense(50))
model.add(Dropout(0.25))
model.add(Dense(10))
model.add(Dropout(0.25))
model.add(Dense(1))

#Optimizer
model.compile(loss='mse',optimizer='adam')

#Number of epoch for training
number_of_epoch=3

#Model training 
history_object=model.fit_generator(train_generator, samples_per_epoch=(len(train_samples)*3)*2, validation_data=validation_generator,nb_val_samples=len(validation_samples), nb_epoch=number_of_epoch)

#Saving model for future use
model.save('model.h5')

# print the keys contained in the history object
print(history_object.history.keys())

# plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()