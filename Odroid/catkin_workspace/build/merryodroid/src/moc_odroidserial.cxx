/****************************************************************************
** Meta object code from reading C++ file 'odroidserial.h'
**
** Created: Sat Nov 8 19:29:33 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/merryodroid/src/odroidserial.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'odroidserial.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OdroidSerial[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      26,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,   26,   31,   31, 0x05,
      32,   57,   31,   31, 0x05,
      70,   97,   31,   31, 0x05,
     112,  138,   31,   31, 0x05,
     152,  180,   31,   31, 0x05,
     196,   57,   31,   31, 0x05,
     221,   97,   31,   31, 0x05,
     248,  138,   31,   31, 0x05,
     274,  180,   31,   31, 0x05,
     302,  331,   31,   31, 0x05,
     348,  360,   31,   31, 0x05,
     365,   31,   31,   31, 0x05,

 // slots: signature, parameters, type, tag, flags
     374,   31,   31,   31, 0x08,
     386,   31,   31,   31, 0x0a,
     398,   31,   31,   31, 0x0a,
     408,   31,   31,   31, 0x0a,
     416,   31,   31,   31, 0x0a,
     431,   31,   31,   31, 0x0a,
     444,   31,   31,   31, 0x0a,
     463,   31,   31,   31, 0x0a,
     474,  492,   31,   31, 0x0a,
     496,  516,   31,   31, 0x0a,
     522,  541,   31,   31, 0x0a,
     546,  569,   31,   31, 0x0a,
     578,   31,   31,   31, 0x0a,
     592,   31,   31,   31, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_OdroidSerial[] = {
    "OdroidSerial\0started(int)\0port\0\0"
    "PIDstoreActualYaw(float)\0uartFloatYaw\0"
    "PIDstoreActualPitch(float)\0uartFloatPitch\0"
    "PIDstoreActualRoll(float)\0uartFloatRoll\0"
    "PIDstoreActualHeight(float)\0uartFloatHeight\0"
    "GUIstoreActualYaw(float)\0"
    "GUIstoreActualPitch(float)\0"
    "GUIstoreActualRoll(float)\0"
    "GUIstoreActualHeight(float)\0"
    "GUIstoreActualBattery(float)\0"
    "uartFloatBattery\0land(float)\0goal\0"
    "disarm()\0send_land()\0startQuad()\0"
    "armQuad()\0start()\0uartListener()\0"
    "uartWriter()\0writeSerialBytes()\0"
    "readPort()\0storePIDYaw(uint)\0yaw\0"
    "storePIDPitch(uint)\0pitch\0storePIDRoll(uint)\0"
    "roll\0storePIDThrottle(uint)\0throttle\0"
    "commandLand()\0setOffset()\0"
};

void OdroidSerial::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        OdroidSerial *_t = static_cast<OdroidSerial *>(_o);
        switch (_id) {
        case 0: _t->started((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->PIDstoreActualYaw((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->PIDstoreActualPitch((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->PIDstoreActualRoll((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->PIDstoreActualHeight((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->GUIstoreActualYaw((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 6: _t->GUIstoreActualPitch((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: _t->GUIstoreActualRoll((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: _t->GUIstoreActualHeight((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 9: _t->GUIstoreActualBattery((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 10: _t->land((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 11: _t->disarm(); break;
        case 12: _t->send_land(); break;
        case 13: _t->startQuad(); break;
        case 14: _t->armQuad(); break;
        case 15: _t->start(); break;
        case 16: _t->uartListener(); break;
        case 17: _t->uartWriter(); break;
        case 18: _t->writeSerialBytes(); break;
        case 19: _t->readPort(); break;
        case 20: _t->storePIDYaw((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 21: _t->storePIDPitch((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 22: _t->storePIDRoll((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 23: _t->storePIDThrottle((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 24: _t->commandLand(); break;
        case 25: _t->setOffset(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData OdroidSerial::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject OdroidSerial::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_OdroidSerial,
      qt_meta_data_OdroidSerial, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OdroidSerial::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OdroidSerial::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OdroidSerial::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OdroidSerial))
        return static_cast<void*>(const_cast< OdroidSerial*>(this));
    return QObject::qt_metacast(_clname);
}

int OdroidSerial::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 26)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 26;
    }
    return _id;
}

// SIGNAL 0
void OdroidSerial::started(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OdroidSerial::PIDstoreActualYaw(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OdroidSerial::PIDstoreActualPitch(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OdroidSerial::PIDstoreActualRoll(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void OdroidSerial::PIDstoreActualHeight(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void OdroidSerial::GUIstoreActualYaw(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void OdroidSerial::GUIstoreActualPitch(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void OdroidSerial::GUIstoreActualRoll(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void OdroidSerial::GUIstoreActualHeight(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void OdroidSerial::GUIstoreActualBattery(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void OdroidSerial::land(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void OdroidSerial::disarm()
{
    QMetaObject::activate(this, &staticMetaObject, 11, 0);
}
QT_END_MOC_NAMESPACE
