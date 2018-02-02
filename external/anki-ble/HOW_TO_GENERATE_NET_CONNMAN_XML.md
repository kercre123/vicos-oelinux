How to generate net.connman.xml
-------------------------------

On the Victor robot, capture the XML outputted from the following
commands:

```
dbus-send --system --type=method_call --print-reply
    --dest=net.connman
    /
    org.freedesktop.DBus.Introspectable.Introspect

dbus-send --system --type=method_call --print-reply
    --dest=net.connman
    /net/connman/technology/wifi
    org.freedesktop.DBus.Introspectable.Introspect
```

Combine the `<interface>` elements of each into `net.connman.xml`.
