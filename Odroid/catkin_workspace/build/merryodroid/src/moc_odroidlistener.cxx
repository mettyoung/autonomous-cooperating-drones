/****************************************************************************
** Meta object code from reading C++ file 'odroidlistener.h'
**
** Created: Sat Nov 8 19:29:34 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/merryodroid/src/odroidlistener.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'odroidlistener.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OdroidListener[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      22,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   35,   44,   44, 0x05,
      45,   60,   44,   44, 0x05,
      64,   82,   44,   44, 0x05,
      88,  105,   44,   44, 0x05,
     110,   44,   44,   44, 0x05,
     121,   44,   44,   44, 0x05,
     134,   44,   44,   44, 0x05,

 // slots: signature, parameters, type, tag, flags
     146,   44,   44,   44, 0x08,
     152,   44,   44,   44, 0x0a,
     160,   35,   44,   44, 0x0a,
     181,   60,   44,   44, 0x0a,
     197,   82,   44,   44, 0x0a,
     215,  105,   44,   44, 0x0a,
     232,  249,   44,   44, 0x0a,
     254,  278,   44,   44, 0x0a,
     284,  308,   44,   44, 0x0a,
     314,  338,   44,   44, 0x0a,
     344,  369,   44,   44, 0x0a,
     376,  278,   44,   44, 0x0a,
     399,  308,   44,   44, 0x0a,
     422,  338,   44,   44, 0x0a,
     445,  369,   44,   44, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_OdroidListener[] = {
    "OdroidListener\0throttleGoal(float)\0"
    "throttle\0\0yawGoal(float)\0yaw\0"
    "pitchError(float)\0pitch\0rollError(float)\0"
    "roll\0quad_arm()\0quad_start()\0quad_land()\0"
    "run()\0start()\0send_throttle(float)\0"
    "send_yaw(float)\0send_pitch(float)\0"
    "send_roll(float)\0send_batt(float)\0"
    "batt\0send_pitch_pterm(float)\0pterm\0"
    "send_pitch_iterm(float)\0iterm\0"
    "send_pitch_dterm(float)\0dterm\0"
    "send_pitch_output(float)\0output\0"
    "send_roll_pterm(float)\0send_roll_iterm(float)\0"
    "send_roll_dterm(float)\0send_roll_output(float)\0"
};

void OdroidListener::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        OdroidListener *_t = static_cast<OdroidListener *>(_o);
        switch (_id) {
        case 0: _t->throttleGoal((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->yawGoal((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->pitchError((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->rollError((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->quad_arm(); break;
        case 5: _t->quad_start(); break;
        case 6: _t->quad_land(); break;
        case 7: _t->run(); break;
        case 8: _t->start(); break;
        case 9: _t->send_throttle((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 10: _t->send_yaw((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 11: _t->send_pitch((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 12: _t->send_roll((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 13: _t->send_batt((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 14: _t->send_pitch_pterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 15: _t->send_pitch_iterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 16: _t->send_pitch_dterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 17: _t->send_pitch_output((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 18: _t->send_roll_pterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 19: _t->send_roll_iterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 20: _t->send_roll_dterm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 21: _t->send_roll_output((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData OdroidListener::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject OdroidListener::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_OdroidListener,
      qt_meta_data_OdroidListener, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OdroidListener::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OdroidListener::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OdroidListener::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OdroidListener))
        return static_cast<void*>(const_cast< OdroidListener*>(this));
    return QObject::qt_metacast(_clname);
}

int OdroidListener::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    }
    return _id;
}

// SIGNAL 0
void OdroidListener::throttleGoal(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OdroidListener::yawGoal(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OdroidListener::pitchError(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OdroidListener::rollError(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void OdroidListener::quad_arm()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}

// SIGNAL 5
void OdroidListener::quad_start()
{
    QMetaObject::activate(this, &staticMetaObject, 5, 0);
}

// SIGNAL 6
void OdroidListener::quad_land()
{
    QMetaObject::activate(this, &staticMetaObject, 6, 0);
}
QT_END_MOC_NAMESPACE
