import csv
import cv2
import numpy as np


lines= []

with open('data/driving_log.csv') as csvfile:
  reader = csv.reader(csvfile)
  for line in reader:
   lines.append(line)

images =[]
measurements = []
for line in lines[1:]:
   for i in range(3):
      source_path=line[i]
      current_path='data/'+(source_path.strip())
      image=cv2.imread(current_path)
      images.append(image)
      if(i==0):
      	measurement=float(line[3])
      elif(i==1):
      	measurement=float(line[3])+.25
      elif(i==2):
      	measurement=float(line[3])-.25

      measurements.append(measurement)


aug_images,aug_measurements=[],[]
for image,measurement in zip(images,measurements):
	aug_images.append(image)
	aug_measurements.append(measurement)
	aug_images.append(cv2.flip(image,1))
	aug_measurements.append(measurement*-1.0)

x_train=np.array(aug_images)
y_train=np.array(aug_measurements)


from keras.models import Sequential,Model
from keras.layers import Flatten,Dense,Lambda,Cropping2D,Convolution2D,Dropout
import matplotlib.pyplot as plt

ch, row, col = 3, 160, 320

model=Sequential()
model.add(Lambda(lambda x: x/127.5 - 1.,input_shape=(row, col,ch),output_shape=(row, col,ch)))
model.add(Cropping2D(cropping=((70,25), (0,0)), input_shape=(row, col,ch)))
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
model.add(Flatten())
model.add(Dense(100))
model.add(Dropout(0.25))
model.add(Dense(50))
model.add(Dropout(0.25))
model.add(Dense(10))
model.add(Dropout(0.25))
model.add(Dense(1))
model.compile(loss='mse',optimizer='adam')
history_object = model.fit(x_train,y_train,validation_split=0.2,shuffle=True,nb_epoch=2,verbose=1)
model.save('model.h5')
### print the keys contained in the history object
print(history_object.history.keys())

### plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()


