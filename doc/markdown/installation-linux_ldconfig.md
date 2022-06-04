### Linux ``ldconfig``

The Force Dimension SDK provides both static (``libdhd.a`` and ``libdrd.a``) and dynamic / shared libraries ([named](https://stackoverflow.com/a/664401) appropriately -- e.g., ``libdhd.so.3.13.2`` and ``libdrd.so.3.13.2``). In order to use the shared libraries, the loader must [know where to find them](https://www.tecmint.com/understanding-shared-libraries-in-linux/) at runtime. This can be accomplished with the following [ldconfig](https://man7.org/linux/man-pages/man8/ldconfig.8.html) command:

```
sudo ldconfig /path/to/force/dimension/sdk/lib/root
```

If this is not done, then an error will be reported when dynamically-linked ROS nodes are started.


