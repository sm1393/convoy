import sys

if __name__ == "__main__":
    print("X_amcl = ", int(sys.argv[2]) + 3)
    print("Y_amcl = ", - int(sys.argv[1]) - 7)
    print("Theta_amcl = Subtact PI/2 from odomTheta")
