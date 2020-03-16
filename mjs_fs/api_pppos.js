
let PPPOS = {
    _imei: ffi('const char* mgos_pppos_get_imei(int)'),
    _iccid: ffi('const char* mgos_pppos_get_iccid(int)'),
    imei: function() {
        return this._imei(0);
    },
    iccid: function() {
        return this._iccid(0);
    },
};