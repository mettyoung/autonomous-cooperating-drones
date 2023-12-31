/****************************************************************************
** Meta object code from reading C++ file 'tcprosstatman.h'
**
** Created: Sat Nov 8 19:29:33 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/merryodroid/src/tcprosstatman.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tcprosstatman.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_TcpRosStatMan[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   26,   26,   26, 0x05,
      27,   26,   26,   26, 0x05,

 // slots: signature, parameters, type, tag, flags
      42,   26,   26,   26, 0x0a,
      50,   26,   26,   26, 0x08,
      57,   26,   26,   26, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TcpRosStatMan[] = {
    "TcpRosStatMan\0connected()\0\0disconnected()\0"
    "start()\0send()\0receive()\0"
};

void TcpRosStatMan::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TcpRosStatMan *_t = static_cast<TcpRosStatMan *>(_o);
        switch (_id) {
        case 0: _t->connected(); break;
        case 1: _t->disconnected(); break;
        case 2: _t->start(); break;
        case 3: _t->send(); break;
        case 4: _t->receive(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData TcpRosStatMan::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject TcpRosStatMan::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TcpRosStatMan,
      qt_meta_data_TcpRosStatMan, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TcpRosStatMan::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TcpRosStatMan::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TcpRosStatMan::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TcpRosStatMan))
        return static_cast<void*>(const_cast< TcpRosStatMan*>(this));
    return QObject::qt_metacast(_clname);
}

int TcpRosStatMan::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void TcpRosStatMan::connected()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void TcpRosStatMan::disconnected()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
