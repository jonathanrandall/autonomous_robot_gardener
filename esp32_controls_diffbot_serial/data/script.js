
// document.addEventListener('DOMContentLoaded',
//     function () {
//         function b(B) {
//             let C; switch (B.type) {
//                 case 'checkbox': C = B.checked ? 1 : 0;
//                     break;
//                 case 'range':
//                 case 'select-one':
//                     C = B.value;
//                     break;
//                 case 'button':
//                 case 'submit':
//                     C = '1';
//                     break;
//                 default:
//                     return;
//             }
//             const D = `${c}/control?var=${B.id}&val=${C}`;
//             fetch(D).then(E => { console.log(`request to ${D} finished, status: ${E.status}`) })
//         }
//         var c = document.location.origin;
//         const e = B => { B.classList.add('hidden') }, f = B => { B.classList.remove('hidden') }, g = B => { B.classList.add('disabled'), B.disabled = !0 }, h = B => { B.classList.remove('disabled'), B.disabled = !1 }, i = (B, C, D) => {
//             D = !(null != D) || D;
//             let E;
//             'checkbox' === B.type ? (E = B.checked, C = !!C, B.checked = C) : (E = B.value, B.value = C), D && E !== C ? b(B) : !D && ('aec' === B.id ? C ? e(v) : f(v) : 'agc' === B.id ? C ? (f(t), e(s)) : (e(t), f(s)) : 'awb_gain' === B.id ? C ? f(x) : e(x) : 'face_recognize' === B.id && (C ? h(n) : g(n)))
//         };
//         document.querySelectorAll('.close').forEach(B => { B.onclick = () => { e(B.parentNode) } }), fetch(`${c}/status`).then(function (B) { return B.json() }).then(function (B) { document.querySelectorAll('.default-action').forEach(C => { i(C, B[C.id], !1) }) });
//         const j = document.getElementById('stream'), k = document.getElementById('stream-container'), l = document.getElementById('get-still'), m = document.getElementById('toggle-stream'), n = document.getElementById('face_enroll'), o = document.getElementById('close-stream'), p = () => { window.stop(), m.innerHTML = 'Start' }, q = () => { j.src = `${c + ':81'}/stream`, f(k), m.innerHTML = 'Stop' };
//         l.onclick = () => { p(), j.src = `${c}/capture?_cb=${Date.now()}`, f(k) }, o.onclick = () => { p(), e(k) }, m.onclick = () => { const B = 'Stop' === m.innerHTML; B ? p() : q() }, n.onclick = () => { b(n) }, document.querySelectorAll('.default-action').forEach(B => { B.onchange = () => b(B) }); const r = document.getElementById('agc'), s = document.getElementById('agc_gain-group'), t = document.getElementById('gainceiling-group'); r.onchange = () => { b(r), r.checked ? (f(t), e(s)) : (e(t), f(s)) }; const u = document.getElementById('aec'), v = document.getElementById('aec_value-group');
//         u.onchange = () => { b(u), u.checked ? e(v) : f(v) };
//         const w = document.getElementById('awb_gain'), x = document.getElementById('wb_mode-group'); w.onchange = () => { b(w), w.checked ? f(x) : e(x) }; const y = document.getElementById('face_detect'), z = document.getElementById('face_recognize'), A = document.getElementById('framesize');
//         A.onchange = () => { b(A), 5 < A.value && (i(y, !1), i(z, !1)) }, y.onchange = () => { return 5 < A.value ? (alert('Please select CIF or lower resolution before enabling this feature!'), void i(y, !1)) : void (b(y), !y.checked && (g(n), i(z, !1))) }, z.onchange = () => { return 5 < A.value ? (alert('Please select CIF or lower resolution before enabling this feature!'), void i(z, !1)) : void (b(z), z.checked ? (h(n), i(y, !0)) : g(n)) }
//     });


var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var socket;
var imsrc1 = 'ws://192.168.1.185:81';
var imsrc2 = 'ws://192.168.1.184:81'


setInterval(() => {
    send_motor_status();
}, 100);

//const json object
var jsonData = '{ "action": "status","value": "stop"}';
var obj_2_update = JSON.parse(jsonData);
var obj_2_send = JSON.stringify(obj_2_update);

function onLoad(event) {
    initWebSocket();
}
function initWebSocket() {
    //const json object

    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
    setup('ws://192.168.1.184:81','FRONT'); //for image
    setup('ws://192.168.1.185:81','REAR'); //for image
}
function onOpen(event) {
    // var jsonData = '{ "action": "status","value": "stop"}';
    // var obj_2_update = JSON.parse(jsonData);
    // var obj_2_send = JSON.stringify(obj_2_update);
    console.log('Connection opened');
    obj_2_update.action = "open"
    obj_2_send = JSON.stringify(obj_2_update);
    try { websocket.send(obj_2_send); }
    catch (e) {
        console.log("onOpen: ")
        console.log(e.message);
    }
}
function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}
function onMessage(event) {
    console.log("onmessage");
    var myObj = JSON.parse(event.data);
    console.log(myObj);
    //asign robot state to myobj state 
    //assign robot speed to myobj speed
    document.getElementById("slider2").value = myObj.speed;
    document.getElementById("textSliderValue").innerHTML = myObj.speed;
    document.getElementById("textRobotState").innerHTML = myObj.status;

    // for (i in myObj.robot_vars) {

    // }
    console.log(event.data);
}
// Send Requests to Control GPIOs
function update_motor_status(element) {
    // var jsonData = '{ "action": "status","value": "stop"}';
    // var obj_2_update = JSON.parse(jsonData);
    // var obj_2_send = JSON.stringify(obj_2_update);

    document.getElementById("textRobotState").innerHTML = element;
    obj_2_update.action = "status";
    obj_2_update.value = element;
    obj_2_send = JSON.stringify(obj_2_update);
    console.log(obj_2_send);
    try { websocket.send(obj_2_send); }
    catch (e) {
        console.log(e.message);
    }

}

function send_move_dist_theta(element, action) {
    // var jsonData = '{ "action": "status","value": "stop"}';
    // var obj_2_update = JSON.parse(jsonData);

    // var obj_2_send = JSON.stringify(obj_2_update);
	//action = "dist" or "theta"
    var text = document.getElementById(element).value;
    
    obj_2_update.action = action;
    obj_2_update.value = text;
    obj_2_send = JSON.stringify(obj_2_update);
    console.log(obj_2_send);
    try { websocket.send(obj_2_send); }
    catch (e) {
        console.log(e.message);
    }

}

function send_motor_status() {

    obj_2_update.action = "update";
    obj_2_update.value = document.getElementById("textRobotState").innerHTML;
    obj_2_send = JSON.stringify(obj_2_update);
    console.log(obj_2_send);
    try { websocket.send(obj_2_send); }
    catch (e) {
        console.log(e.message);
    }

}

function update_motor_speed(element) {
    var sliderValue = document.getElementById(element.id).value;
    // var jsonData = '{ "action": "speed","value": "0"}';
    // var obj_2_update = JSON.parse(jsonData);

    obj_2_update.action = "speed";

    document.getElementById("textSliderValue").innerHTML = sliderValue;
    obj_2_update.value = sliderValue;
    obj_2_send = JSON.stringify(obj_2_update);
    console.log(obj_2_send);
    websocket.send(obj_2_send);

}

function setup(host, el) {
    // var host = 'ws://192.168.1.184:81';
    // set_on_message_off(imsrc1);
    // if(socket) socket.terminate();

    // img.src = "";
    // set_on_message_off(imsrc2);

    socket = new WebSocket(host);


    socket.binaryType = 'arraybuffer';
    if (socket) {
        socket.onopen = function () {
        }
        socket.onmessage = function (msg) {
            var img;
            try {
                img = document.getElementById(el);
            }
            catch (error) {
                console.log('returning');
                 return; }
            var bytes = new Uint8Array(msg.data);
            var binary = '';
            var len = bytes.byteLength;
            for (var i = 0; i < len; i++) {
                binary += String.fromCharCode(bytes[i])
            }
            try{ img.src = 'data:image/jpg;base64,' + window.btoa(binary);} catch(e){}

        }
        socket.onclose = function () {
            showServerResponse('The connection has been closed.');
        }
    }
}

function set_on_message_off(imsrcx) {
    var socket = new websocket(imsrcx);
    socket.terminate();
    // if(socket){
    //     socket.onmessage = function(msg){}
    // }
}

// Function to get and update GPIO states on the webpage when it loads for the first time
function getStates() {
    websocket.send("states");
}

window.addEventListener('load', onLoad);

// jQuery(function ($) {
//     if (!('WebSocket' in window)) {
//         alert('Your browser does not support web sockets');
//     } else {
//         setup();
//     }
//     function setup() {
//         var host = 'ws://192.168.1.119:81';
//         var socket = new WebSocket(host);
//         socket.binaryType = 'arraybuffer';
//         if (socket) {
//             socket.onopen = function () {
//             }
//             socket.onmessage = function (msg) {
//                 var bytes = new Uint8Array(msg.data);
//                 var binary = '';
//                 var len = bytes.byteLength;
//                 for (var i = 0; i < len; i++) {
//                     binary += String.fromCharCode(bytes[i])
//                 }
//                 var img = document.getElementById('live');
//                 img.src = 'data:image/jpg;base64,' + window.btoa(binary);
//             }
//             socket.onclose = function () {
//                 showServerResponse('The connection has been closed.');
//             }
//         }
//     }
// });
