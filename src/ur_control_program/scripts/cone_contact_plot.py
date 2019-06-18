#!/usr/bin/env python
import rospy
#plot contact map
import matplotlib.pyplot as plt 
import matplotlib.animation as animation


def animate(i,xs,ys):
  ax.plot(xs,ys,color='b')
  ax.set_xlim(-1.5, 1.5)
  ax.set_ylim(0, 2.5)

def main():
	rospy.init_node('cone_contact_plot', anonymous=True)

  #subscibe to the topic published by the control script
  bumper_subscriber = rospy.Subscriber('/oblique_cone/bumper', ContactsState, self.bumper_handler)



if __name__ == '__main__':
	main()