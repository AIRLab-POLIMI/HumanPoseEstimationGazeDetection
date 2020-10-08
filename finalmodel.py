import keras.backend as K
import keras as keras

import numpy as np

from keras.models import Sequential, Model
from keras.layers import Dense, LocallyConnected1D, Flatten, Input, Multiply, Concatenate, Lambda

from keras import regularizers

def my_init(shape, dtype=None):
    
    initVec = K.variable(np.ones(shape)*0.02)
    K.set_value(initVec[:,:-1],K.eval(K.random_uniform((shape[0],2),minval=-0.1, maxval=0.1)))

    return initVec

def prepare_modelSingle(activ_func):

    inputX = Input(shape=(15,1,))

    modelX = Sequential()
    modelX.add(Lambda(lambda x: x[:,0:10:2],input_shape=(15,1)))
    modelX.add(LocallyConnected1D(filters=1,kernel_size=(1),strides=1,kernel_initializer='ones', activation=activ_func, bias_initializer='ones'))
    modelX.add(Flatten())

    modelY = Sequential()
    modelY.add(Lambda(lambda x: x[:,1:11:2],input_shape=(15,1)))
    modelY.add(LocallyConnected1D(filters=1,kernel_size=(1),strides=1,kernel_initializer='ones', activation=activ_func, bias_initializer='ones'))
    modelY.add(Flatten())

    # confidence pairs
    modelC = Sequential()
    modelC.add(Lambda(lambda x: 2.8532*(x[:,10:15]-0.5383),input_shape=(15,1)))
    modelC.add(LocallyConnected1D(filters=1,kernel_size=(1),strides=1,kernel_initializer='ones', kernel_regularizer=regularizers.l2(0.001), activation='sigmoid', use_bias=False))
    modelC.add(Flatten())

    modelX.build()
    #modelX.summary()

    modelC.build()
    #modelC.summary()

    x1 = modelX(inputX)
    y1 = modelY(inputX)
    
    c1 = modelC(inputX)

    xMult = Multiply()(([x1,c1]))
    yMult = Multiply()(([y1,c1]))

    xyMerge = Concatenate()(([xMult,yMult]))

    d0 = Dense(10, kernel_initializer='he_normal', kernel_regularizer=regularizers.l2(0.0001), activation=activ_func)(xyMerge)
    d1 = Dense(10, kernel_initializer='he_normal', kernel_regularizer=regularizers.l2(0.0001), activation=activ_func)(d0)
    out = Dense(3, kernel_initializer=my_init)(d1)
    model = Model(inputs=inputX, outputs=out)
    #model.summary()   

    return model