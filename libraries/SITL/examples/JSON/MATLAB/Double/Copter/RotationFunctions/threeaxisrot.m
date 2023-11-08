function r1r2r3 = threeaxisrot(r11, r12, r21, r31, r32)
r1 = atan2(r11, r12);
r2 = asin(r21);
r3 = atan2(r31, r32);

r1r2r3 = [r1, r2, r3];