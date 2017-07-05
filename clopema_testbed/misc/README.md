# CloPeMa Misc

Here we store miscellaneous resources.


## Udev Rules (clopema.rules)

Is example of the udev rule that assigns symlinks to certain devices such as
nikon cameras.

In order to make it work one must change to serial numbers accordingly and copy
the file under `/etc/udev/rules.d/` with the name starting with number for the
correct order of execution, such as `10-clopema.rules`.
