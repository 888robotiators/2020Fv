# This can probably be sent to the Graveyard

# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
npzFile = np.load('cam0M.npz')
dist = npzFile['dist']
mtx = npzFile['mtx']
newcameramtx = npzFile['newcameramtx']
roi = npzFile['roi']

camera = cv2.VideoCapture(0)



# keep looping
while True:
	# grab the current frame
	(grabbed, frame) = camera.read()
        frame2 = frame.copy()
        h, w = frame.shape[:2]
        #newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        r = frame.copy()
        r[:,:,0] = 0
        r[:,:,1] = 0

        g = frame.copy()
        g[:,:,0] = 0
        g[:,:,2] = 0

        b = frame.copy()
        b[:,:,1] = 0
        b[:,:,2] = 0

        #lowerWhite = np.array([0, 0, 190])
        #upperWhite = np.array([100, 200, 255])   
        lowerWhite = np.array([0, 100, 190])
        upperWhite = np.array([25, 220, 255])
        lowerRed = np.array([0,0, 170])
        upperRed = np.array([100,100, 255])
        redMask = cv2.inRange(frame, lowerRed, upperRed)   
	frame2 = frame.copy()
        #perform hsv mask on image
        mask = cv2.inRange(hsv, lowerWhite, upperWhite)
	# show the frame to our screen
	gray = cv2.cvtColor(r, cv2.COLOR_BGR2GRAY)
        #def onmouse(k, x, y, s, p):
            #global frame
            #if k==1:
                #print frame[y,x]
        #cv2.namedWindow("frame")
        #cv2.setMouseCallback("frame",onmouse)
        contours, heirarchy = cv2.findContours(redMask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        #print len(contours)
        testPoints = np.zeros((2*2,3), np.float32)
        testPoints[:,:2] = np.mgrid[0:2, 0:2].T.reshape(-1,2)
        #print testPoints
        #print ""
        bigC = None
        #cv2.line(frame2, (frame2.shape[1] / 2, 0), (frame2.shape[1] /2, frame2.shape[0]), (255, 0, 0), 2)
        #print mtx
        #objP = np.array([(4, 2.5, 0), (-4, 2.5, 0), (-4, -2.5, 0), (4, -2.5, 0)], dtype = np.float32())
        #objP = np.array([(4, 2.5, 0), (-4, 2.5, 0), (-4, -2.5, 0), (4, -2.5, 0),(2, 2.5, 0), (-2, 2.5, 0), (-2, -2.5, 0), (2, -2.5, 0)], dtype = np.float32())
     	objP = np.array([(-4, 2.5, 0), (-4, -2.5, 0),(-2, 2.5, 0), (-2, -2.5, 0),(2, 2.5, 0) , (2, -2.5, 0), (4, 2.5, 0), (4, -2.5, 0)], dtype = np.float32())
	
        #objP = objP * 25.4
        #objP = np.array([(1, 1, 0), (-1, 1, 0), (-1, -1, 0), (1, -1, 0)], dtype = np.float32())
        #print objP
        test = np.zeros((4,3), np.float32)
        #print test
        #print type(objP)
        #print type(objP[0][0])
        #axis = np.float32([[0,0,0],[0,3,0],[3,3,0],[3,0,0],[0,0,-3],[0,3,-3],[3,3,-3], [3,0,-3] ])
        #axis = np.float32([[-4,-2.5,0],[-4,2.5,0],[4,2.5,0],[4,-2.5,0],[-4,-2.5,-3],[-4,2.5,-3],[4,2.5,-3], [4,-2.5,-3] ])

        
        if len(contours) > 0:
                bigC = None
                #allRect = cv2.minAreaRect(contours)
                #trueBox = cv2.cv.BoxPoints(allRect)
                #trueBox = np.int0(box)
                for c in contours:
                        epsilon = 0.1*cv2.arcLength(c,True)
                        approx = cv2.approxPolyDP(c,epsilon,True)
                        #print approx
                        cv2.drawContours(frame, [approx], 0, (255, 255, 255), 3)
                        if bigC == None:
                                bigC = c.copy()
                                allApprox = approx.copy()
                        else:
                                bigC = np.concatenate((bigC,c), axis=0)
                                allApprox = np.concatenate((allApprox, approx), axis=0)
        	
                #bigRect = cv2.minAreaRect(bigC)
                #(x,y,w,h) = cv2.boundingRect(bigC)
                #bigEpsilon = 0.1*cv2.arcLength(bigC,True)
                #bigApprox = cv2.approxPolyDP(bigC,epsilon,True)
  
                #cv2.drawContours(frame2, [bigApprox], 0, (255, 0, 255), 3)
                #cv2.drawContours(frame2, [allApprox], 0, (255, 0, 255), 3)
                #cv2.rectangle(frame2, (x,y), (x+w,y+h), (255,0,0),2)
                #print type(bigRect)
                #bigBox = cv2.cv.BoxPoints(bigRect)
                #bigBox = np.int0(bigBox)
	        #cv2.drawContours(frame2,[bigBox],0,(255,0,255),2)
                #cv2.drawContours(frame2,[trueBox],0,(255,0,255),2)
                #print bigBox[0]
                
                distance = 18
                #focalWidth = 750.9799
                focalWidth = 669
                #focalLength = (bigRect[1][0] * distance) / 8
                #print focalLength
                #print bigBox[0]
                #cv2.circle(frame2, bigBox[0], 5, (0, 0, 255), 1)  #yellow first point
                #cv2.circle(frame2, bigBox[1], 5, (255, 0, 255), 1)  #yellow first point
                #cv2.circle(frame2, bigBox[2], 5, (0, 255, 0), 1)  #yellow first point
                #cv2.circle(frame2, bigBox[3], 5, (255, 255, 0), 1)  #yellow first point
                if len(allApprox) == 8:
                        #print len(allApprox)
                        #cv2.circle(frame2, (bigBox[0][0], bigBox[0][1]), 4, (0, 255, 255), -1) #Yellow Point "Bottom Point"
                        #cv2.circle(frame2, (bigBox[1][0], bigBox[1][1]), 4, (0, 0, 255), -1)   #Red Point "Left Point"
                        #cv2.circle(frame2, (bigBox[2][0], bigBox[2][1]), 4, (0, 255, 0), -1)   #Green Point "Top Point"
                        #cv2.circle(frame2, (bigBox[3][0], bigBox[3][1]), 4, (255, 0, 0), -1)   #Blue Point "Right Point"
                        #print bigBox
                        #bigBox = bigBox.astype(np.float32)
                        #allApprox = allApprox.astype(np.float32)
                        #print "Not sorted"
                        
                        #print allApprox[0][0:8]
                        #sortApprox = allApprox.copy()
			#print allApprox
                        sortApprox = sorted(allApprox, key=lambda k: [k[:,0], k[:,1]])
                        sortApprox = np.array(sortApprox)
                        #print sortApprox
                        #print sortApprox[1][0]
                        #print len(sortApprox[:,0])
                        #print sortApprox[:,0]
                        for i in range(len(sortApprox[:,0])):
                                if i % 2 == 0:
                                        approx1 = sortApprox[i][0].copy()
                                        approx2 = sortApprox[i+1][0].copy()
                                        if (approx1[1] < approx2[1]):
                                                sortApprox[i][0] = approx2.copy()
                                                sortApprox[i+1][0] = approx1.copy()
                        #print allApprox[:,0]
		        #for s in sortApprox:
                                #print s
                        #print ""
                        #print allApprox[0]
                        #sortApprox.sort()
                        #print "Sorted"
                        #print sortApprox
                        #print allApprox
                        #print bigBox
                        #print type(sortApprox)
                        #print type(allApprox)
                        #print type(allApprox[0])
                        #print type(allApprox[0][0])
                        #print allApprox
                        #print sortApprox
                        sortApprox = np.array(sortApprox)
                        #print allApprox
                        #print sortApprox

                        cv2.drawContours(frame2, [sortApprox], 0, (255, 0, 255), 3)
                        sortApprox = sortApprox.astype(np.float32)
                        retval, rvec, tvec = cv2.solvePnP(objP, sortApprox, mtx, None)
                        #imgpoints, jac = cv2.projectPoints(axis, rvec, tvec, mtx, None)
                        #print mtx
                        #print ""
                        #print newcameramtx
                        #print ""
                        #print sortApprox
                        #imgpoints = np.int32(imgpoints).reshape(-1,2)
                        #print imgpoints
                        #print objP
                        #print bigBox
                        #print imgpoints[:4]
                        #cv2.drawContours(frame2, [imgpoints[:4]], -1, (0,255,0),-3)
                        #for i,j, in zip(range(4),range(4,8)):
                                #cv2.line(frame2, tuple(imgpoints[i]), tuple(imgpoints[j]), (255),3)
                        #cv2.drawContours(frame2, [imgpoints[4:]], -1, (0,0,255),3)
                        #print rvec
                        #print ""
                        #print tvec
                        #print ""
                        inches = tvec[2]
                        cv2.putText(frame2, "%.2fft" % (inches / 12), (frame2.shape[1]-200, frame2.shape[0]-20), cv2.FONT_HERSHEY_TRIPLEX, 2.0, (0, 255, 0), 3)
                #if bigRect [1][0] != 0:
                        #print bigRect
                        #print bigBox
                        #print axis
                        #inches = (8 *focalWidth) / (bigRect[1][0])
                
	#cv2.imshow("frame", frame)
        cv2.imshow("frame2", frame2)
        #cv2.imshow("red", r)
        #cv2.imshow("blue", b)
        #cv2.imshow("green", g)
         
        
        cv2.imshow("RedMask", redMask)
        #cv2.imshow("green", g)
        #cv2.imshow("blue", b)
        #cv2.imshow("gray", gray)
        #cv2.imshow("hsv mask", mask)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
