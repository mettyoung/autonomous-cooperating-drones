/****************************************************************************
** Meta object code from reading C++ file 'prcontroller.h'
**
** Created: Sat Nov 8 19:29:31 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/merryodroid/src/prcontroller.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'prcontroller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PRController[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,   26,   30,   30, 0x05,
      31,   44,   30,   30, 0x05,
      50,   63,   30,   30, 0x05,
      69,   82,   30,   30, 0x05,
      88,  103,   30,   30, 0x05,

 // slots: signature, parameters, type, tag, flags
     110,   30,   30,   30, 0x0a,
     118,   30,   30,   30, 0x0a,
     132,   30,   30,   30, 0x08,
     138,   30,   30,   30, 0x08,
     150,  168,   30,   30, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PRController[] = {
    "PRController\0getPWM(uint)\0pwm\0\0"
    "pTerm(float)\0pterm\0iTerm(float)\0iterm\0"
    "dTerm(float)\0dterm\0sOutput(float)\0"
    "output\0start()\0togglePause()\0run()\0"
    "setOffset()\0storeError(float)\0myGoal\0"
};

void PRController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PRController *_t = static_cast<PRController *>(_o);
        switch (_id) {
        case 0: _t->getPWM((*reinterpret_cast< uint(*)>(_a[1]))); break;
        case 1: _t->pTerm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->iTerm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->dTerm((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->sOutput((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->start(); break;
        case 6: _t->togglePause(); break;
        case 7: _t->run(); break;
        case 8: _t->setOffset(); break;
        case 9: _t->storeError((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PRController::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PRController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_PRController,
      qt_meta_data_PRController, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PRController::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PRController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PRController::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PRController))
        return static_cast<void*>(const_cast< PRController*>(this));
    return QObject::qt_metacast(_clname);
}

int PRController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void PRController::getPWM(unsigned int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PRController::pTerm(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PRController::iTerm(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PRController::dTerm(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void PRController::sOutput(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
