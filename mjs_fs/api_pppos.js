let PPPOS = {
  _imei: ffi('char* mgos_pppos_get_imei_raw(int)'),
  _imsi: ffi('char* mgos_pppos_get_imsi_raw(int)'),
  _iccid: ffi('char* mgos_pppos_get_iccid_raw(int)'),

  imei: function() {
    return this._imei(0);
  },

  imsi: function() {
    return this._imsi(0);
  },

  iccid: function() {
    return this._iccid(0);
  },

};
