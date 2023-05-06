from car import mycar

car = mycar()

print("Press f, b, l, r to go, q : stop")
while True:
    key = input("Where : ")
    if key == 'f':
        car.goForward(0.5)
    elif key == 'b':
        car.goBackward(0.5)
    elif key == 'r':
        car.goRight(0.5)
    elif key == 'l':
        car.goLeft(0.5)
    elif key == 'q':
        car.stop()
    elif key == 'x':
        break