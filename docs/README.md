## Documenting:
Documentation should be used as a blueprint for development, then filled in when specifics are established. 

The documentation of this repository follows a package level scheme. As shown below:
```
samples
├── README.md ──────────────> overview(component)
├── cpp
│   ├── producer
│   │   ├── README.md ──────> most importantly(package)
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   ├── include
│   │   ├── launch
│   │   ├── package.xml
│   │   ├── src
│   │   └── test
├── python
├── sample_msgs
└── samples_diagram.svg
```

### Component Level 
- Overview of component as a whole

### Package Level   
- **Purpose** – What the package does and its role within the component.  
- **Inputs & Outputs** – Data flow, including message types, service calls, or file interactions.  
- **Key Features** – Key classes, nodes, or scripts, along with their relationships.  
- **Usage** – How to build, run, and test the package.  
- **Configuration** – Relevant parameters, environment variables, or dependencies.

### System Architecture 
The system architecture can be viewed in this [document](Architecture_Map.pdf), along with the .odg file (use libre draw to edit).

### Infrastructure Documentation
1. [Project Infrastructure Development Docs](https://github.com/WATonomous/wato_monorepo/tree/main/docs/dev/)
2. [ROS Structure Docs](src/samples/README.md)