/****************************************************************************
** Meta object code from reading C++ file 'telemetry.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.0.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/telemetry.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'telemetry.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.0.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Telemetry_t {
    QByteArrayData data[25];
    char stringdata[448];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_Telemetry_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_Telemetry_t qt_meta_stringdata_Telemetry = {
    {
QT_MOC_LITERAL(0, 0, 9),
QT_MOC_LITERAL(1, 10, 3),
QT_MOC_LITERAL(2, 14, 0),
QT_MOC_LITERAL(3, 15, 19),
QT_MOC_LITERAL(4, 35, 23),
QT_MOC_LITERAL(5, 59, 7),
QT_MOC_LITERAL(6, 67, 19),
QT_MOC_LITERAL(7, 87, 17),
QT_MOC_LITERAL(8, 105, 21),
QT_MOC_LITERAL(9, 127, 18),
QT_MOC_LITERAL(10, 146, 20),
QT_MOC_LITERAL(11, 167, 18),
QT_MOC_LITERAL(12, 186, 20),
QT_MOC_LITERAL(13, 207, 20),
QT_MOC_LITERAL(14, 228, 22),
QT_MOC_LITERAL(15, 251, 21),
QT_MOC_LITERAL(16, 273, 19),
QT_MOC_LITERAL(17, 293, 20),
QT_MOC_LITERAL(18, 314, 21),
QT_MOC_LITERAL(19, 336, 19),
QT_MOC_LITERAL(20, 356, 20),
QT_MOC_LITERAL(21, 377, 15),
QT_MOC_LITERAL(22, 393, 15),
QT_MOC_LITERAL(23, 409, 18),
QT_MOC_LITERAL(24, 428, 18)
    },
    "Telemetry\0run\0\0adjustBatteryLevel1\0"
    "merryMsg::Msg::ConstPtr\0battery\0"
    "adjustBatteryLevel2\0actualCoordinates\0"
    "on_btn_height_clicked\0on_btn_yaw_clicked\0"
    "on_btn_start_clicked\0on_btn_arm_clicked\0"
    "on_btn_reset_clicked\0on_btn_print_clicked\0"
    "on_btn_legends_clicked\0on_btn_start1_clicked\0"
    "on_btn_arm1_clicked\0on_btn_land1_clicked\0"
    "on_btn_start2_clicked\0on_btn_arm2_clicked\0"
    "on_btn_land2_clicked\0quad1_connected\0"
    "quad2_connected\0quad1_disconnected\0"
    "quad2_disconnected\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Telemetry[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  119,    2, 0x08,
       3,    1,  120,    2, 0x08,
       6,    1,  123,    2, 0x08,
       7,    0,  126,    2, 0x08,
       8,    0,  127,    2, 0x08,
       9,    0,  128,    2, 0x08,
      10,    0,  129,    2, 0x08,
      11,    0,  130,    2, 0x08,
      12,    0,  131,    2, 0x08,
      13,    0,  132,    2, 0x08,
      14,    0,  133,    2, 0x08,
      15,    0,  134,    2, 0x08,
      16,    0,  135,    2, 0x08,
      17,    0,  136,    2, 0x08,
      18,    0,  137,    2, 0x08,
      19,    0,  138,    2, 0x08,
      20,    0,  139,    2, 0x08,
      21,    0,  140,    2, 0x0a,
      22,    0,  141,    2, 0x0a,
      23,    0,  142,    2, 0x0a,
      24,    0,  143,    2, 0x0a,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void, 0x80000000 | 4,    5,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Telemetry::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Telemetry *_t = static_cast<Telemetry *>(_o);
        switch (_id) {
        case 0: _t->run(); break;
        case 1: _t->adjustBatteryLevel1((*reinterpret_cast< const merryMsg::Msg::ConstPtr(*)>(_a[1]))); break;
        case 2: _t->adjustBatteryLevel2((*reinterpret_cast< const merryMsg::Msg::ConstPtr(*)>(_a[1]))); break;
        case 3: _t->actualCoordinates(); break;
        case 4: _t->on_btn_height_clicked(); break;
        case 5: _t->on_btn_yaw_clicked(); break;
        case 6: _t->on_btn_start_clicked(); break;
        case 7: _t->on_btn_arm_clicked(); break;
        case 8: _t->on_btn_reset_clicked(); break;
        case 9: _t->on_btn_print_clicked(); break;
        case 10: _t->on_btn_legends_clicked(); break;
        case 11: _t->on_btn_start1_clicked(); break;
        case 12: _t->on_btn_arm1_clicked(); break;
        case 13: _t->on_btn_land1_clicked(); break;
        case 14: _t->on_btn_start2_clicked(); break;
        case 15: _t->on_btn_arm2_clicked(); break;
        case 16: _t->on_btn_land2_clicked(); break;
        case 17: _t->quad1_connected(); break;
        case 18: _t->quad2_connected(); break;
        case 19: _t->quad1_disconnected(); break;
        case 20: _t->quad2_disconnected(); break;
        default: ;
        }
    }
}

const QMetaObject Telemetry::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Telemetry.data,
      qt_meta_data_Telemetry,  qt_static_metacall, 0, 0}
};


const QMetaObject *Telemetry::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Telemetry::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Telemetry.stringdata))
        return static_cast<void*>(const_cast< Telemetry*>(this));
    return QWidget::qt_metacast(_clname);
}

int Telemetry::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 21;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
