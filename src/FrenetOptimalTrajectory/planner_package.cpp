#include "planner_package.h"
#include <structmember.h>
#include "py_cpp_struct.h"

/**
 * FotPlanner Defintions
 **/
PyObject *FotPlanner_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    FotPlanner *self = (FotPlanner *)type-> tp_alloc(type, 1);
    self->fot = NULL; // set fot = nullptr
    printf("Module: planner new\n");
    return (PyObject *) self;
}

// need to modify it to work with initial conditions later
int FotPlanner_init(PyObject *self, PyObject *args, PyObject *kwds) {
    // pass IC and HP into this
    PyObject *ic = NULL;
    PyObject *hp = NULL;
    if (PyArg_UnpackTuple(args, "args", 1, 2, &ic, &hp)) {
        ((FotPlanner *) self)->fot = new AnytimeFrenetOptimalTrajectory((FrenetInitialConditions *) ic, (FrenetHyperparameters *) hp); 
        printf("Module: planner init\n");
        return 0;
    } else{
        return -1;
    }
}

void FotPlanner_dealloc(FotPlanner *self) {
    printf("Module: planner dealloc\n");
    delete self->fot;
    Py_TYPE(self)->tp_free(self);
}

static PyObject *method_async_plan(PyObject *self, PyObject *args) {
    ((FotPlanner *) self)->fot->asyncPlan();
    printf("Module: Async Plan\n");
    // return self;
    return Py_None;
}

// for ease for testin for now,
// we just return a float representing the cost of it
static PyObject *method_get_path(PyObject *self, PyObject *args) {
    printf("Module: Get Path\n");
    // FotPath *rv = new FotPath();
    printf("Got path\n");
    // rv->path = ((FotPlanner *) self)->fot->getBestPath();
    // return (PyObject *) rv;
    return Py_None; // place holder for now
    // need to modify this to return a FrenetPath
}

static PyObject *method_stop_plan(PyObject *self, PyObject *args) {
    ((FotPlanner *) self)->fot->stopPlanning();
    printf("Module: Stop Plan\n");
    return Py_None;
}

PyMethodDef FotPlanner_methods[] = {
    {"async_plan", method_async_plan, METH_VARARGS, "Start Async Planning"}, 
    {"get_path", method_get_path, METH_VARARGS, "Get Best Path"},
    {"stop_plan", method_stop_plan, METH_VARARGS, "Stop Planning, Shut Down all Workers"},
    {NULL, NULL, 0, NULL},
};


PyTypeObject FotPlannerType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "fot_planner.FotPlanner",
    .tp_basicsize = sizeof(FotPlanner),
    .tp_dealloc = (destructor)FotPlanner_dealloc,
    // .tp_repr = (reprfunc)FotPlanner_repr,
    .tp_flags = Py_TPFLAGS_DEFAULT |
    Py_TPFLAGS_BASETYPE,
    .tp_doc = "fot_planner.FotPlanner objects",
    .tp_methods = FotPlanner_methods,
    // .tp_members = FotPlanner_members,
    // .tp_as_mapping
    // .tp_init = (initproc)FotPlanner_init, // we don't need init here?
    .tp_new = FotPlanner_new
};

static struct PyModuleDef fot_planner_module = {
    PyModuleDef_HEAD_INIT,
    "fot_planner",
    "Anytime Fot Planner",
    -1,
    FotPlanner_methods
};

/* Initialize the fot_planner module */
PyMODINIT_FUNC PyInit_fot_planner(void) {
    Py_Initialize();

    PyObject* m;

     if (PyType_Ready(&FotPlannerType) < 0)
        return NULL;
        
    m = PyModule_Create(&fot_planner_module);
    if (m == NULL)
        return NULL;
    // do we need reference count?
    // Py_INCREF(&FotPlannerType);

    PyModule_AddObject(m, "FotPlanner", (PyObject *)&FotPlannerType);
    printf("FOT Planner Module imported!\n");
    fflush(stdout);
    return m;
}