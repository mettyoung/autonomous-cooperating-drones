/****************************************************************************
** Meta object code from reading C++ file 'heightcontroller.h'
**
** Created: Sat Nov 8 19:29:31 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/merryodroid/src/heightcontroller.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'heightcontroller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_HeightController[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   30,   34,   34, 0x05,

 // slots: signature, parameters, type, tag, flags
      35,   34,   34,   34, 0x0a,
      43,   34,   34,   34, 0x0a,
      57,   34,   34,   34, 0x08,
      63,   34,   34,   34, 0x08,
      75,   92,   34,   34, 0x08,
      99,  118,   34,   34, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_HeightController[] = {
    "HeightController\0getPWM(uint)\0pwm\0\0"
    "start()\0togglePause()\0run()\0setOffset()\0"
    "storeGoal(float)\0myGoal\0storeActual(float)\0"
    "myActual\0"
};

void HeightController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        HeightController *_t = static_cast<HeightController *>(_o);
        switch (_id) {
        case 0: _t->getPWM((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 1: _t->start(); break;
        case 2: _t->togglePause(); break;
        case 3: _t->run(); break;
        case 4: _t->setOffset(); break;
        case 5: _t->storeGoal((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 6: _t->storeActual((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData HeightController::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject HeightController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_HeightController,
      qt_meta_data_HeightController, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &HeightController::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *HeightController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *HeightController::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_HeightController))
        return static_cast<void*>(const_cast< HeightController*>(this));
    return QObject::qt_metacast(_clname);
}

int HeightController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void HeightController::getPWM(unsigned int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
