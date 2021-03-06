## issofa_attachconstraint


### New features


### Improvements
- Add a position, velocity and response factor in order to correct the constraint.

### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_beamfemff


### New features
- apply rigid transform on first beam
- anisotropy on beams model

### Improvements


### Cleans
- young modulus was in "ReadOnly", now set as "SetRequired" (same as poisson ratio)

### Bugfixes


### Moved files

------------------------------------------------

## issofa_bugfix


### New features


### Improvements


### Cleans
- BaseObject: Output message in serr for required datas. Error word should be reserved for messages that will make the application fail.
- RayTriangleVisitor and SceneLoaderFactory: clean warnings
- ParticlesRepulsionForceField: Add empty implementation of addKToMatrix to get rid of console warnings.

### Bugfixes
- AttachConstraint: fix crash when indices are resized
- BTDLinearSolver: fix constant log + operator new [] vs operator delete mismatch
- CatmullRomSplineMapping: fix applyJ compilation
- ConstantForceField: fix avoid crashes in draw when trying to apply a constantforcefield to a non existing point
- DistanceGrid: fix incorrect use of serr
- GeneralConstraintSolver: fix incorrect parent class in SOFA_CLASS macro
- IndicesFromValues: fix use getTemplateName to avoid problems of links with other data
- EulerImplicitSolver: fix Disable calls to solveConstraint from EulerImplicitSolver by default as it is not needed unless an very specific solver is added and it currently crashes in other cases + wrong argument order in calls to `AdvancedTimer::stepNext()`
- FixedConstraint and PartialFixedConstraint: fix "fixed FixedConstraint & PartialFixedConstraint for size-varying mechanical objects".
- MechanicalObject: fix crash in debug with null pointer
- Mass: fix remove error logging in Mass method that are inherited from Forcefield API and that are not relevant for Mass
- MechanicalMatrixVisitor and MechanicalOperations: fix if using a Linear Solver, projective constraints were wrongly applied when a force field is in a child node
- Mesh2PointMechanicalMapping: fix constraints ApplyJT
- Metis(gk_arch.h): fix compilation with Visual Studio 2015
- ParticlesRepulsionForceField and RepulsiveSpringForceField: fix avoid division by 0 on repulsion force fields
- PersistentContactBarycentricMapping: fix init variables in constructor
- RestShapeSpringsForceField: fix Runtime stiffness tunning was not working + optimisation and cleaning
- SkinningMapping: fix compilation of SofaRigid
- SofaViewer: fix crashes when current camera of pick-handler is NULL
- SurfacePressureForceField: fix volume computation formula
- TaitSurfacePressureForceField: fix contribution to the stiffness matrix for the second component df = P+dN in TaitSurfacePressureForceField
- TopologicalMapping: fix display error messages when a TopologicalMapping failed to be created
- TriangularFEMForceFieldOptim: fix principal values ordering with input matrix is diagonal + uninitialized value warning
- VisualModelImpl: fix wrong object (triangles) called when adding quads in handleTopologyChange()
- VoxelGridLoader: `std::unique` result was not used

### Moved files

------------------------------------------------

## issofa_build

### New features


### Improvements
- RunSofa does now its own find_package for standalone compilation.
- Update csparse includes in SofaSparseSolver library.
- Activate the compilation of some more Triangle FEM components forregression testing purposes.

### Cleans

### Bugfixes
- Fix the detection of sixense SDK path on Linux.
- Fix SofaPhysicsAPI must initialize all sofa modules.
- Fix missing link of SofaGraphComponent in SofaOpenglVisual's CMakeList for windows.

### Moved files
------------------------------------------------

## issofa_clean

### New features

### Improvements
- Add default constructor on struct HaptionData in Haption plugin.
- Add default constructor of OmniData in SensableEmulation plugin. 
- Add missing implementation of `Base::removeData()` in Base. 


### Cleans
- Clean the use of constant complexity function & prefix operators.
- Clean uninitialized variables in constructors. 
- Clean potentials access to *(NULL). 
- Clean uses of const& instead of const. 
- Clean missing initialization of member's variables in somes constructors.
- Clean using of wrong type in some printf. 
- Clean prefix operators and variables initialization with non-used values.
- SofaGuiQt: Reduce the scope of variables and remove unsused instructions. 
- LinearSolverConstraintCorrection: Clean by removing getOdeSolver() method, use `getContext::get()` directly.
- DeformableOnRigidFrameMapping: remove repartition and indexFromEnd logic which was never used, nor tested. Also improve the coding style : use d_ prefix for members that are data, prefer Accessors to beginEdit/endEdit methods.



### Bugfixes
- QtOgreViewer: Fix warnings.
- SensableEmulation : Fix build error. 
- Sensable : Fix use of vector namespace. 
- Coil.scn : Fix the use of wrong parameters and paths. Coil demo is now working. Moreover update use of input/output instead of object1/object2 in mapping of InstrumentCoil.
- BlenderExporter: Fix missing or incorrect include guards.
- TriangleModel: Fix missing or incorrect include guards.
- Fix warnings with gcc.

### Moved files

------------------------------------------------

## issofa_collision


### New features
- RuleBasedContactManager: no more need to declare the list of variables within the "variables" Data.
- TriangleModel and LineModel: add accessors to rest position.

### Improvements
- **[WARNING]**  DetectionOutputVector: Move factory methods outside the class.

### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_constraintsolving

### New features
- Add correctionVelocityFactor and correctionPositionFactor Data in UncoupledConstraintCorrection to experiment with modulating the constraint response influence on visual tool positions (UNTESTED).
- Add more detail in timing of GenericConstraintSolver.

### Improvements
- Constraint problem locking extended to be safer (but not yet completely safe) when using multiple forcefeedback devices.
- GenericConstraintCorrection: automatically search LinearSolver from context when solverName is not specified (same behavior as LinearSolverConstraintCorrection).
- Factorize the computation of the compliance matrix and virtualize the computation of dx.
- Improve UncoupledConstraintCorrection compliance computation. Use the same factors for the compliance matrix computation and the correction computation as the one used in LinearSolverConstraintCorrection.


### Cleans
- Clean and Optimize UncoupledConstraintCorrection, factorize code between rigids and vecs, optimize addComplianceInConstraintSpace significantly for large number of constraints.
- Move new constraint accumulation visitors from GenericConstraintSolver.h to MechanicalVisitor.h so that they can be used by other solvers.
- Code refactoring in GenericConstraintCorrection.

### Bugfixes
- Fix FreeMotionAnimationLoop allocates the freeposition / freevelocity as first thing in the time step.
- Fix Haptic mode with haptic influence from constraints applied on all frames. It was not working well on the simulation side.
- Add a correction velocity factor and correction position factor as data in UncoupledConstraintCorrection.
- Fix, the GenericConstraintSolver was not providing the correct number of interations for profiler stats.
- GenericConstraintCorrection's ODESolver is now searched locally.
- Fix the Local Jacobian Matrix. It was not resized when no more constraints were applied on a MechanicalState.
- UncoupledConstraintCorrection for rigid is now in a 7 value vector. When using the default initialization of the compliance using the mass matrix of the rigid the linear part of the compliance was added twice.


### Moved files
- Move contact identifier code in a separate file to avoid inelegant FrictionContact.h inclusion in each contact class file.


------------------------------------------------

## issofa_datainit


### New features
- TopologyDataHandler: add helper functions tu use on types without default constructor.

### Improvements
- Base: Detect bad calls and invalid conversions in `Base::initData0`.

### Cleans
- **[WARNING]** Data: removing default constructor. All Data now need to be initialized in all constructors.

### Bugfixes


### Moved files

------------------------------------------------

## issofa_debug

### New features
- Add a vector_access_failure looks at SOFA_DEBUG_VECTOR_ASSERT environment variable to control if assert() is called (enabled by default).


### Improvements
- Change verbose display of EulerImplicitSolver to be able to compare two executions more easily.
- Update display node name in addition of node type on xml node creation failure for easier debugging.
- Update: CMake option SOFAFRAMEWORK_CHECK_CONTAINER_ACCESS_FAILURE to activate access failure detection inside Sofa containers.


### Cleans


### Bugfixes
- Fix the export of graphviz graphs with multi-mappings, topological mappings, slaves, engines and loaders.


### Moved files

------------------------------------------------

## issofa_deprecatedapi


### New features


### Improvements
- BaseConstraint: improve API. getConstraintResolution is now a pure virtual method.
- BaseConstraint: make `ConstraintResolution::resolution()` abstract to detect and fix implementations using the deprecated API.

### Cleans


### Bugfixes
- PartialFixedConstraint: fix applyConstraint API.

### Moved files

------------------------------------------------

## issofa_draw


### New features
- ConstantForceField: add two time parameters to command the start and the end of the applied force.
- ConstantForceField: add loopTime and handleTopologyChange parameters.

### Improvements
- RestShapeSpringsForceField: add display and render RestShapeSpringsForceField used as fixed constraints with spheres.
- TriangularFEMForceFieldOptim: activate stress visualization for triangles that are under TriangularFEMForcefieldOptim influence.

### Cleans


### Bugfixes
- PointSetGeometryAlgorithms: draw of the topology items indices even if the bbox of the base is flat or invalid.

### Moved files

------------------------------------------------

## issofa_engines


### New features
- MathOp: add * and / operations to vectors.
- MathOp: perform at each time step if the listening is set to true.
- BoxROI: add OutROI data for all primitive types.
- SphereROI: add isPartOf[...]InSphere methods + add strictlyInROI flag.

### Improvements
- IndexValueMapper: change template from DataTypes to VecT and include all common VecTypes.
- MergeVectors: vectors of unsigned int can now be merged with MergeVectors.

### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_fasttriangularbendingff


### New features


### Improvements
- FastTriangularBendingSprings: add unit test in SofaGeneralDeformable_test.
- FastTriangularBendingSprings: add possibility to define the rest position of the mesh and to avoid bending effect.
- FastTriangularBendingSprings: add quadratic bending model for Inextensible Surfaces.

### Cleans


### Bugfixes
- FastTriangularBendingSprings: bending spring was using a wrong formula.

### Moved files

------------------------------------------------

## issofa_gui


### New features


### Improvements
- set animate flag before calling `GUIManager::setScene` so that GUI can use it
- runSofa now forwards nbIterations to all GUI and not only Batch

### Cleans


### Bugfixes

### Moved files

------------------------------------------------

## issofa_mask
- This category is not about mask, but improves were coupled with mask modifications not reported.

### New features


### Improvements


### Cleans


### Bugfixes
- Fix compilation errors in mass. 

### Moved files

------------------------------------------------

## issofa_meshspringff


### New features


### Improvements
- MeshSpringForceField: add drawing of springs elongation 

### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_multithreading

### New features
- Add logGraphUpdates / logDataUpdates global variable in objectmodel to be able to debug multi-threading issue (not declared in headers, temporary until a better design is proposed)
- Add a flag d_threadsafevisitor in solvers to disable writing in mechanicalstates of InteractionForceField in MechanicalVReallocVisitor/MechanicalVFreeVisitor as it might crash in multithreaded context
- Add `Contact::computeResponse()` and `Contact::finalizeResponse()` to separate thread-safe from sequential parts of the response computations
- Implement thread-safe Data engine/links updates by adding a lock while calling update().This requires that requestUpdate() or requestUpdateIfDirty() be called instead of update() directly, which is now protected.Bonus side effect is that it no longer matter where or if engines call cleanDirty() in their update method.
- Update all engines to new thread-safe design
- Add execution info in Task and index in WorkerThread
- Add support for multithreaded task logging (for visual profiling)
- Add some const correctness in getColor and getName methods.

### Improvements
- Remove ClassInfo, deprecated in favor of BaseClass, and make sure all static instances are initialized while registering objects in the factory, to FIX "pure virtual call" crashes with visual c++ in multi-threaded contexts
- Replace `Creator<Contact::Factory,` with `sofa::core::collision::ContactCreator<` to improve thread safety of contact creation.
- AnimateBeginEvent and AnimateEndEvent can now receive an optional pointer to a TaskStatus ( in a form of a void pointer ). When using SofaMultithreading, this taksStatus pointer can be used to spawn the AnimateBeginEvent method in a separate thread. It is the reponsibility of the component to decide whether or not its AnimateBeginEvent method is thread safe.
- Change BaseObject argument parsing. When parsing, ensures if src attribute is used with link @ now allow the use of src="". Topologies container associated to src="" are supposed to be filled by following topological mapping. Using src="@" to avoid link warning drove to an autolink where the loader refers on container itself. Modification of implicated scenes and xml accordingly.
- WorkerThread maintains a list of stealable tasks as well as a list of higher priority thread specific tasks. Remove ThreadSpecificTask (not used and redundant with new thread-specific task queues). Update Task logging.
- Adapted MultiThreading plugin to the new build system.
- SetDirectory no longer changes the system current directory (shared by all threads in the process), but uses a global variable to record the current directory to be used within Sofa. But changing to the current file's directory is needed to load Python scene. For the moment, python loader is not use with multithreading, changing directory in the loader is a temporary solution. The functioning of python scene loader need to be adapted in the futur.

### Cleans
- Streamline calls to createResponse in case no groupManager is used
- Clean warnings in Task and TaskSchedulerBoost for Multithreading plugin.

### Bugfixes
- Fix crash with multithreaded collision detection. Call `TriangleModel::getTriangleFlag` method for all triangles of the topology to force the initialization of adjacency.information in the topology before some actual collision detection code is executed.
- DDGNode should never be copied to ensure thread safe. Add private copy constructor that gives compilation error to ensure that.
- Compilation fixes for compatibility with boost 1_55. Corrected also the copyright notice.
- Fix execution freeze bug due to simultaneous and reciprocal task stealing between 2 threads (WorkerThread).
- Fix data race at startup, WorkerThread:mTaskScheduler was set by the main thread while being read by the worker thread.

### Moved files

------------------------------------------------

## issofa_optimization


### New features
- **[WARNING]** BaseContactMapper: add setConstraintMode virtual method to set ForcesMapped and MassesMapped to false in the mappings of the contacts based on constraints.
- MatSym: add constructors and invertMatrix for 1x1 and 2x2 cases.
- MatrixLinearSolver: add CRS 1x1 instantiations (useful to debug bloc-based solvers).
- SparseMatrix: add find method.
- CompressedRowSparseMatrix: add keepEmptyRows flag to specify whether empty rows should be kept in the rowIndex/rowBegin vectors.

### Improvements
- GenericConstraintSolver: Perform ConstraintCorrection computations only if it belongs to an active node.
- PrecomputedLinearSolver: optimize compliance computation.
- SparseLDLSolver: optimize compliance computation.
- SparseLDLSolver: output more info to files if asked + factorization is recomputed if orderingMode is changed.
- **[WARNING]** SparseLDLSolver: use CSR 3x3 bloc matrix by default (as it is much faster in the common case of 3D dofs).
- BTDLinearSover: improve performances with AddJMinvJt method (20x speedup for this method).
- BlocMatrixWriter: add new methods to support masses matrix computation and optimize adding blocs to symmetric matrices and adding blocs with values on the diagonal only.
- FastTriangularBendingSprings: optimize addKToMatrix with BlocMatrixWriter new methods.
- CompressedRowSparseMatrix: compress() is now more efficient when no new value appeared in the matrix.
- StiffSpringForceField: improve addKToMatrix computation (when it is attached to a single state).

### Cleans


### Bugfixes
- BlocMatrixWriter: fix crash in addSym* with visual c++ (writeable bloc pointers can get invalidated by asking for a second bloc if it resizes the underlying vector).
- CompressedRowSparseMatrix: use Real instead of Bloc for ref value in filterValues().
- CompressedRowSparseMatrix: use Bloc size instead of Scalar size in fullRows() and fullDiags().

### Moved files

------------------------------------------------

## issofa_otherfeatires


### New features
- BarycentricMapping : Add accessor to the mapping internal data and to the topology used by the input dofs
- StiffSpringForceField : Add accessor to the spring stiffness matrix.
- DeformableOnRigidFrameMapping : add ption to use the reverse transform of the rigid frame.
- Quater : createFromRotationVector calls now createFromExp

### Improvements
- Quaternion log method algorithm and rename the associated method
- quatToRotationVector is now an alias of getLog
- DeformableOnRigidFrameMapping : Remove repartition and indexFromEnd logic which was never used, nor tested. Also improve the coding style : use d_ prefix for members that are data, prefer Accessors to beginEdit/endEdit methods.
- DeformableOnRigidFrameMapping : disable mapping accumulation when d_invertRigidFrame is toggled instead of setting the mapping as non mechanical.
- Regressoin_test : generate new reference for BeamFEMForceField, there is a progression with feature about quaternion for small angles.


### Cleans
- DeformableOnRigidFrameMapping : removed unused option globalToLocalCoords.
- Quater : clean code duplication

### Bugfixes
- DeformableOnRigidFrameMapping : fix compilation error

### Moved files

------------------------------------------------

## issofa_planeff


### New features
- PlaneForceField: make it possible to apply the PlaneForceField on a subset of points indicated in a new indices Data
- PlaneForceField: can define a different height for each point

### Improvements


### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_plugins


### New features
- Add a suffixMap for RequiredPlugin to specifie each library suffix according to the build flag to be more flexible.

### Improvements


### Cleans
- Extra logs in RequiredPlugin logging.

### Bugfixes


### Moved files

------------------------------------------------

## issofa_python


### New features
- Add binding python to get slaves and names on baseObjects.
- Add Python bindings for triangle and point topology modifiers. Example scene which show triangle refining.
- Creation of Binding_BaseTopologyObject common to PointSetTopologyModifier and TriangleSetTopologyModifier.
- Add a getObjects() method to python BaseContext interface. Allow selection of objects based on type and name. May be extended to include more search options later.

### Improvements
- SendScriptEvent should only be applied on a specified node, not on root !
- WARNING: may affect python script behavior.
- Allow to get the value of a data as a string when its type is not handle by the binding.
- Object and type names are now both optional when calling BaseContext_getObjects(). Moreover the search direction can now optionally be passed to BaseContext_getObjects() (default is 'Local').
- Binding_Data expose fullPath read only parameter for BaseData that derive from DataFileName, which as the name suggests, returns the fullPath to the file pointed by that data. Returns None otherwise.

### Cleans

### Bugfixes
- If the data to be gotten using the bind is a vector of type 'unknown' then the text of the data is returned as a whole.
- Fix compilation and warnings with gcc.
- Fix SofaPython build on linux.
- Fix undefined behaviour and potential crash in Python BaseContext.getObjects() due to PyList accessed out of bounds. 
- Fix a memory leak which happens whenever an object is transferred from C++ to Python.

### Moved files
------------------------------------------------

## issofa_rigidbody


### New features
- add GyroscopicVelocityImpulseConstraint: projective constraint that computes the angular velocity impulse relative to the gyroscopic term using an implicit equation.
- add RigidMass unit test
- add RigidMassComplianceEngine: engine which computes a compliance based on a rigid mass and a position of the rigid mechanical object, to be able to compute the inverse of the inertia mass matrix in global coordinates.
- EllipsoidForceField : - Rigid" are taken into account in this class - "addKToMatrix" methodology are added in this class - New variable "nbContact" which is used for "EyeRotationSpring" class in order to use correct "complianceOutput" and "stiffnessOutput"
- UniformRigidMass: can define a uniform mass value for all dofs of a Rigid MechanicalObject, or a specific mass value for each dof


### Improvements
- RigidMass_test: test for conversion from local to global coordinates for inertia matrix.

### Cleans
- EllipsoidForceField: add empty getPotentialEnergy
- UniformRigidMass: - no longer inherits from ProjectiveConstraint.- remove unused method computeInvInertiaMassMatrixWorld from the rigid mass component since it should never be called her

### Bugfixes
- fix RigidTypes: RigidMass cannot be converted silently to a scalar value anymore.
- fix UniformRigidMass: bug (rigidMass[i] being accessed instead of the ref handling case where i is not part of the array and 0 is used)
- fix RigidMassComplianceEngine: - data d_rigidComplianceUnzipped must be specified as an output of this engine.- removed the h^2 term when assembling the compliance from the inertia mass matrix to ensure homogeneity between uncoupled and linearSolver constraint correction classes- use mass^-1 when computing the inverse of InertiaMassMatrix

### Moved files

------------------------------------------------

## issofa_sofaphysicsapi


### New features
- File (no extension) or class name: message

### Improvements
- SofaPhysicsOutputMesh : implement the interface to access edges in a visual model using the SofaPhysicsAPI

### Cleans
- Add `ShaderElement::ShaderValueType` enum to find the type of the shader attribute, and methods getSEValueType() / getSEValuePointer() to retrieve the type and the values. Implement it in OglAttribute and put Ogl*Attribute constructors and destructors as protected to for the use of SPtr and New. Use the new methods in SofaPhysicsAPI to expose vertex attribute data is a cleaner way.

### Bugfixes
- `SofaPhysicsOutputMesh::getNbAttributes()` now support attributes that are not only a single float per vertex 
- SofaPhysicsSimulation : fix memory leak

### Moved files

------------------------------------------------

## issofa_solvers


### New features
- EuleurImplicitSolver : add projectForce flag to apply projection constraints to stored force vector
- SparseLDLSolver : add orderingMode data to be able to control at runtime which permutation computation is used

### Improvements


### Cleans


### Bugfixes
- look for METIS library in SofaGeneral

### Moved files

------------------------------------------------

## issofa_sph


### New features


### Improvements


### Cleans
- SPHFluidForceField and SPHKernel: factorize the SPHKernel class and its methods
- SPHKernel:  license and include guard

### Bugfixes


### Moved files

------------------------------------------------

## issofa_springff


### New features
- SpringForceField : add enabled flag in `SPringFF::Spring`

### Improvements
- SpringForceField : make all Data public to be able to edit the springs into from other components

### Cleans


### Bugfixes


### Moved files

------------------------------------------------

## issofa_subsetmultimapping


### New features
- SubsetMultiMapping : add ability to have interlaced indices in identity mode
- SubsetMultiMapping : add indexPairs initialisation from a vector that only contians the parent indices

### Improvements


### Cleans
- SubsetMultimapping : use ReadAccessors, log error messages in case of invalid indices

### Bugfixes
- SubsetMultimapping : compilation with gcc (according to standard, a vector of ints is automatically filled with 0 by the constructor)
- SubsetMultimapping : check indices to prevent invalid access (temporary fix)
- SubsetMultimapping : remove duplicated line

### Moved files

------------------------------------------------

## issofa_tests


### New features
- DataTypeInfoTest in framework_test
- SparseMatrix add units tests

### Improvements
- SofaEngine_test. Remove dependency to SofaTest in TransformEngine_test so that it can be executed at compile time just after SofaEngine is built

### Cleans


### Bugfixes
- Warnings in DataTest.cpp

### Moved files

------------------------------------------------

## issofa_topology


### New features
- UncoupledConstraintCorrection : add option to disable handling of topological changes for compliance vector
- Container : add accessor to the topology type
- SubsetMapping : add resizeToModel flag in SubsetMapping to enable resizing of the destination MechanicalState to the size of indices
- BaseContext : add new method getActiveMeshTopology to get relevant topology, taking `BaseMaping::sameTopology()` into account
- PointSetTopology : add some unit tests

### Improvements
- each time a topological change sequence is detected in BarycentricMapperTriangleSetTopology the mapping is initialized
- PointSetTopologyContainer maintains a list of point indices, accessible through the data points
- Topology : Factorize the last method specific to each topological elements in data containers

### Cleans
- MechanicalObject size and link to topology is now exposed as Data and Link, using the new getActiveMeshTopology() method. Set the link manually and/or set useTopology="false" to override

### Bugfixes
- apply data container updates before mapping updates
- use existing parameter handleTopologyChange in SubsetMapping
- slightly cleanup topology change handling method for triangle mappers
- check of upper topology based on the emptiness of the upper container
- crash in `MeshTopology::init()` for non manifold meshes
- re-enable Propagate of topological changes inbetween triangle creation and removal in `TriangleSetTopologyModifier::addRemoveTriangles(),` as it is required in cases involving chains of topology mapping and topology data containers depending on each other 

### Moved files

------------------------------------------------

## issofa_vectorid

### New features
- integer_id : Add a DataTypeInfo for vector_id to allow vector_id data link in the xml files

### Improvements


### Cleans


### Bugfixes
- integer_id : Fix compilation with gcc 

### Moved files

------------------------------------------------

## issofa_visitors


### New features


### Improvements
- API Change : remove option to apply projective constraints during velocity and/or position propagation visitors, as it makes it unreliable to put projective constraints in child nodes (required when they apply on the DOFs but with a different/subset topology). All codes (solvers and animationloop) must now explicitly call projection operations/visitors before propagations when required (mostly after `OdeSolver::solve())`

### Cleans


### Bugfixes


### Moved files

------------------------------------------------
