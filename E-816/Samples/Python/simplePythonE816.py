import win32com.client
import time
import string

E816 = win32com.client.Dispatch( 'E816_COM.E816' )

ret = E816.ConnectRS232(2,115200)

if ret:	
	(ret, idn) = E816.qIDN("")
	print idn
	(ret, axes) = E816.qSAI("")
	print "axes :" + axes

	axis = axes[0]

	print "SVO " + axis +" 1"
	E816.SVO(axis, [1])

	print "MOV " + axis +" 0"
	E816.MOV(axis, [0.0])
	time.sleep(1)
	(ret, pos) = E816.qPOS(axis, [0.0])
	print "Current position: " + str(pos[0])
		
	print "MOV " + axis +" 10"
	E816.MOV(axis, [10.0])
	time.sleep(1)
	(ret, pos) = E816.qPOS(axis, [0.0])
	print "Current position: " + str(pos[0])

	for i in range(64):
		E816.SWT(axis, i, i)
	
	for i in range(64):
		(ret, val) = E816.qSWT(axis, i, 0)
		print str(i)+": "+str(val)

	E816.CloseConnection()
else:
	print "Connection failed!"