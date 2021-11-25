import codecs
import numpy as np
from matplotlib import pyplot as plt
np.set_printoptions(threshold=np.inf)
file1=open("hummingbirdpayorien_0.txt","r")
#file2=open("hummingbirdpaydespos_0.txt","r")
data1=file1.read()
#data2=file2.read()
#print data1[2]
data1=np.fromstring(data1, dtype=float, sep="\n")
#data2=np.fromstring(data2, dtype=float, sep="\n")
#print data1
#print data2
data3=[data1[0]]
#data4=[data2[2]]
#if len(data1) < len(data2):
len1=len(data1)
#else:
   #len1=len(data2)
len1=int(len1)
for i in range(0,len1,1):
   #print type(i)
   data3=np.concatenate((data3,[data1[i]]))
   #data4=np.concatenate((data4,[data2[i]]))
   #print data3
   #print data4
#print data1
x=np.linspace(0,len(data3)/100,len(data3))
#print len(data3)
plt.plot(x,data3,color='r',label='pay_orien_err')
#plt.plot(x,data4,color='g',label='pay_zdpos')
plt.legend()
plt.show()
file1.close()
#file2.close()
