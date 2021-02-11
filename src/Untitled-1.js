fechainit = global.get("fecha_inicio"); //fecha inicio
modo = msg.payload.modo;  //estado modo
proxriego = global.get("proxriego");  //proximo riego
riego_ok = global.get("riego_ok");  //bool para validar riego hecho
savedata = global.get("fechaguardar");

var aux, aux2;

msg1 = {};
msg2 = {};
msg3 = {};

riegohumus = {};
actualizar = {};

//-------------FECHA HOY-------------------------
var date2 = new Date();
var mes2 = date2.getMonth();
var ano2 = date2.getFullYear();
var dia2 = date2.getDate();
if(mes2==12){
    mes2 = date2.getMonth()-11;
}
else mes2 = date2.getMonth()+1;
//-------------FECHA INICIO-------------------------
var date = new Date(fechainit);
var mes = date.getMonth();
var ano = date.getFullYear();
var dia = date.getDate();

if(mes==12){
    mes = date.getMonth()-11;
}
else mes = date.getMonth()+1;
//--------------FECHA PROXIMA------------------------
var date3 = new Date(proxriego);
var mes3 = date3.getMonth();
var ano3 = date3.getFullYear();
var dia3 = date3.getDate();

if(mes3==12){
    mes3 = date3.getMonth()-11;
}
else mes3 = date3.getMonth()+1;
//--------------------------------------

var fechahoy = dia2 + "/" + mes2 + "/" + ano2;
var fechainicio = dia + "/" + mes + "/" + ano;
var fechasiguiente = dia3 + "/" + mes3 + "/" + ano3;
var next = new Date(dia3+"/"+mes3+"/"+ano3);

if (typeof savedate !== 'undefined'){
    if(modo && riego_ok==false && fechasiguiente !=savedata){
        if ((dia3==dia2) && (mes3==mes2) && (ano3==ano2) && (aux==false)){
            riego_ok = true;
            savedate = fechasiguiente;
            global.set("savedate",savedata);
            global.set("riego_ok",riego_ok);
            riegohumus.payload = true;
            actualizar.payload = proxriego;
            return [riegohumus,actualizar,null];
        }
    }
}
else{
    if(modo && riego_ok==false){
        riego_ok = true;
        savedata = fechainicio;
        global.set("fechaguardar",savedata)
        riegohumus.payload = true;
        return [riegohumus,null,null];
    }
}



