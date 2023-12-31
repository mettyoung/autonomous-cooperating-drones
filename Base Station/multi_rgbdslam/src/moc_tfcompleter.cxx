/****************************************************************************
** Meta object code from reading C++ file 'tfcompleter.h'
**
** Created: Thu Oct 23 18:16:51 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "tfcompleter.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tfcompleter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_TfCompleter[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x0a,
      21,   12,   12,   12, 0x0a,
      37,   35,   12,   12, 0x0a,
      66,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TfCompleter[] = {
    "TfCompleter\0\0start()\0togglePause()\0m\0"
    "setMapToMap2(Transformation)\0run()\0"
};

void TfCompleter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TfCompleter *_t = static_cast<TfCompleter *>(_o);
        switch (_id) {
        case 0: _t->start(); break;
        case 1: _t->togglePause(); break;
        case 2: _t->setMapToMap2((*reinterpret_cast< Transformation(*)>(_a[1]))); break;
        case 3: _t->run(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData TfCompleter::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject TfCompleter::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TfCompleter,
      qt_meta_data_TfCompleter, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TfCompleter::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TfCompleter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TfCompleter::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TfCompleter))
        return static_cast<void*>(const_cast< TfCompleter*>(this));
    return QObject::qt_metacast(_clname);
}

int TfCompleter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
