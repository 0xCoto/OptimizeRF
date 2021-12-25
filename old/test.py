import threading
import motor1
import motor2
import motor3
import time

goto1 = -44000
goto2 = -20000
goto3 = -20000

move1 = threading.Thread(target=motor1.drive, args=(goto1,))
move2 = threading.Thread(target=motor2.drive, args=(goto2,))
move3 = threading.Thread(target=motor3.drive, args=(goto3,))
move1.start()
move2.start()
move3.start()
print('Moving...')
move1.join()
move2.join()
move3.join()
print('All moves complete')
time.sleep(2)
print('Moving back to 0 everything')
move1 = threading.Thread(target=motor1.drive, args=(0,))
move2 = threading.Thread(target=motor2.drive, args=(0,))
move3 = threading.Thread(target=motor3.drive, args=(0,))
move1.start()
move2.start()
move3.start()
print('Moving...')
move1.join()
move2.join()
move3.join()
print('All moves complete')
