field1 = 'x';           value1 = Xyzti(logical(li), 1);
field2 = 'y';           value2 = Xyzti(logical(li), 2);
field3 = 'z';           value3 = Xyzti(logical(li), 3);
field4 = 'intensity';   value4 = Xyzti(logical(li), 5);

s = struct(field1, value1, field2, value2, ...
    field3, value3, field4, value4);

mat2las(s, strcat('-o C:\Users\jsa\Desktop\04_EXPORT\Nikkila\161124_162516_VUX-1HA_cracks_and_holes2.las'));

