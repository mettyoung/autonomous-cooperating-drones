/****************************************************************************
** Meta object code from reading C++ file 'localplanner.h'
**
** Created: Sat Nov 15 20:18:42 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/localplanner.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'localplanner.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LocalPlanner[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      18,   14,   13,   13, 0x05,
      39,   34,   13,   13, 0x05,
      62,   57,   13,   13, 0x05,
      89,   79,   13,   13, 0x05,
     119,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
     136,   13,   13,   13, 0x0a,
     144,   13,   13,   13, 0x0a,
     156,   13,   13,   13, 0x0a,
     175,   13,   13,   13, 0x08,
     181,   14,   13,   13, 0x08,
     209,  203,   13,   13, 0x08,
     233,   57,   13,   13, 0x08,
     256,   13,   13,   13, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_LocalPlanner[] = {
    "LocalPlanner\0\0yaw\0goalYaw(double)\0"
    "goal\0goalPitch(double)\0roll\0"
    "goalRoll(double)\0pitch,yaw\0"
    "setCommandQuad(double,double)\0"
    "requestNewPath()\0start()\0hoverQuad()\0"
    "toggleAutonomous()\0run()\0send_yaw_goal(double)\0"
    "pitch\0send_pitch_goal(double)\0"
    "send_roll_goal(double)\0runAutonomous()\0"
};

void LocalPlanner::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LocalPlanner *_t = static_cast<LocalPlanner *>(_o);
        switch (_id) {
        case 0: _t->goalYaw((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->goalPitch((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->goalRoll((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->setCommandQuad((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: _t->requestNewPath(); break;
        case 5: _t->start(); break;
        case 6: _t->hoverQuad(); break;
        case 7: _t->toggleAutonomous(); break;
        case 8: _t->run(); break;
        case 9: _t->send_yaw_goal((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->send_pitch_goal((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->send_roll_goal((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->runAutonomous(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LocalPlanner::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LocalPlanner::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_LocalPlanner,
      qt_meta_data_LocalPlanner, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LocalPlanner::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LocalPlanner::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LocalPlanner::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LocalPlanner))
        return static_cast<void*>(const_cast< LocalPlanner*>(this));
    return QObject::qt_metacast(_clname);
}

int LocalPlanner::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void LocalPlanner::goalYaw(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LocalPlanner::goalPitch(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LocalPlanner::goalRoll(double _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void LocalPlanner::setCommandQuad(double _t1, double _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void LocalPlanner::requestNewPath()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}
QT_END_MOC_NAMESPACE
