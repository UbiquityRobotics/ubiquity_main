---
layout: post
title: Building A Better Robot
---
Every robot application has a few basic needs: Mobility, Navigation, Power, and Compute. Servicing each of those needs requires significant knowledge and experience in different areas. The main challenge while designing Hercules was embedding meaningful capabilities in each of these categories while keeping the cost of the platform down.

Mobility is one of the most fundamental aspects of any robot platform, being able to move around an environment while carrying a significant payload is something robots should be great at. Our goal for Hercules is being able to drive around in any ADA (Americans with Disabilities Act) compatible area while carrying a payload upwards of 50 pounds (23 kg). Other robot platforms such as the [Turtlebot](http://turtlebot.com) have difficulty carrying much more than 11 pounds (5 kg) on hard floors, and even less payload on carpet. To achieve such a high payload on Hercules we have gone for large [Brushless DC Motors](http://en.wikipedia.org/wiki/Brushless_DC_electric_motor) that are embedded inside the wheels. These motors provide lots of power and high reliability, for a relatively low cost.

The navigation puzzle is one that has been 
