# PIC24-I2C

I2C Driver written for the purpose of the BYU ME 495R Mechatronics Class by [gellings](https://github.com/gellings), [emros](https://github.com/emros), and [superjax](https://github.com/superjax).  This was originally written as the firmware for a robot capable of shooting ping-pong balls into a goal indicated by Infrared LED beacons, and used two Wii-mote IR sensors like the those found [here](http://www.dfrobot.com/index.php?route=product/product&product_id=1088) to get position and angle meaurements to the IR LED.

This repo is a stripped-down version of the code, featuring only the i2c communications, infrared vision functions and drivers for the camera.  The full code and hardware schematics can be found [here](https://github.com/TeamBaymax).  The stripped down version of the code is intended to help other students succeed in future mechatronics competitions in the future by supplying quick driver functions for i2c devices, but if it is helpful for other classes at other universities, we would love your feedback.  It has been heavily commented to aid in understanding of the files.

Documentation can be found on this repo's [wiki](https://github.com/superjax/PIC24-I2C/wiki) (see right column of page)

<script>
  (function(i,s,o,g,r,a,m){i['GoogleAnalyticsObject']=r;i[r]=i[r]||function(){
  (i[r].q=i[r].q||[]).push(arguments)},i[r].l=1*new Date();a=s.createElement(o),
  m=s.getElementsByTagName(o)[0];a.async=1;a.src=g;m.parentNode.insertBefore(a,m)
  })(window,document,'script','//www.google-analytics.com/analytics.js','ga');

  ga('create', 'UA-55384642-6', 'auto');
  ga('send', 'pageview');

</script>