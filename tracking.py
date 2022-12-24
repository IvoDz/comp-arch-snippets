# Python skripts sejas un tās kustības noteikšanai
# Autors Ivo, Dzalbs
# Pēdējo reizi mainīts : 24.12.2022

# Izmanto OpenCV bibliotēku, lai video plūsmā atpazītu cilvēka seju, un ziņotu, ja novērojamas ievērojamas kustības.
# Izmantota arī pyserial bibliotēka (serial), kas nodrošina iespēju savienoties ar Arduino un sūtītu signālu par kustībām.
# Arduino daļa palika iekomentēta, jo patlaban tāda nav, un kods ir tīri prototipēšanas nolūkos, lielās bildes saprašanai.
# Problēma ir tajā, ka optimālai darbībai nepieciešams vienkrāsains fons un laba kamera. Ja šo apstākļu nav, bieži vienu jebkurš 'noise' tiek uzskatīts par kustību
# Protams, ražošanā algoritms tiktu optimizēts, šis ir primitīvs piemērs
# 
# Visa informācija atrodama OpenCV un pyserial bibliotēku dokumentācijās, konkrēti:
# https://docs.opencv.org/3.4/d6/d00/tutorial_py_root.html
# https://pyserial.readthedocs.io/en/latest/shortintro.html#opening-serial-ports 

import cv2 as cv
#import serial

# Izveido seriālo savienojumu ar Arduino
# ser = serial.Serial('COM3', 9600)

# Definējam kadrus, kuriem skatīsimies starpību un noteiksim, vai seja kustas.
frame1 = None
frame0 = None

# Izveido Haar Cascade objektu (šis atpazīs seju)
face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Palaižam kameru 
video_capture = cv.VideoCapture(0) # 0 - primārā kamera (testēts uz klēpjdatora)

while True:
    # Nepārtrauktā ciklā ielasām video plūsmu kadrs pa kadram
    ret, frame = video_capture.read()

    # Augstākai precizitātei mainām kadra formātu uz grayscale - pelēkiem toņiem.
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Nosaka sejas kadrā.
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # faces - masīvs ar noteiktām sejām, ja kādu noteicām, sūtam signālu uz Arduino
    #if len(faces) > 0: ser.write(b'1')   
    #else: ser.write(b'0')
        

    # Testa nolūkiem - ap seju zīmē taisnstūri. (x,y,w,h) - koordinātas.
    for (x, y, w, h) in faces:
        cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    
    # Tālāk - kods sejas kustības noteikšanai.
    frame1 = frame0
    frame0 = frame

    if frame1 is not None and frame0 is not None:
        # Atrod starpību, starp tagadējo un iepriekšējo kadru, pārveido tos grayscale, izveido kontūru kolekciju.
        diff = cv.absdiff(frame1, frame0)
        gray_diff = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
        thresh = cv.threshold(gray_diff, 25, 255, cv.THRESH_BINARY)[1]
        contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Pārbauda, vai kāda no kontūrām ir seja
        for contour in contours:
            # Atrod kontūras laukumu
            (x, y, w, h) = cv.boundingRect(contour)
            if w > 50 and h > 50:
                print("Seja kustas!")
                # Sūtam ziņu, ka seja kustas
                # ser.write(b'1')

    # Rāda kadru
    cv.imshow('Video', frame)

    # spiest q lai izvairītos no bezgalīga cikla - beigšanas nosacījums.
    if cv.waitKey(1) == ord('q'):
        break

# Beidzam savienojumu, atvienojamies no Arduino
video_capture.release()
#ser.close()
cv.destroyAllWindows()