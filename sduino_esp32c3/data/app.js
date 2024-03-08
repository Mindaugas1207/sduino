
var thumbWidth = 20;

function setBubble(range, bubble) {
    var newValue = Number(((range.value - range.min) * (range.clientWidth - thumbWidth) / (range.max - range.min)) - (bubble.clientWidth / 2) + (thumbWidth / 1.6));
    bubble.innerHTML = range.value;
    if (bubble.getAttribute('data-defval') === null) { bubble.setAttribute('data-defval', range.value); }
    var backValue = (range.value - range.min) / (range.max - range.min) * 100;
    range.style.background = `linear-gradient(to right, Teal 0%, Cyan ${backValue}%, LightGray ${backValue}%, white 100%)`;
    bubble.style.left = `${newValue}px`;
}

function updateRange(range_wrap, value) {
    var range = range_wrap.querySelector('.range');
    var bubble = range_wrap.querySelector('.bubble');
    range.value = value;
    setBubble(range, bubble);
}

function updateButton(button, value) {
    button.value = value;
    button.style.backgroundColor = value > 0 ? 'green' : 'red';
}

function parseEntry(entry) {
    var [id, value] = entry;
    var entity = document.getElementById(id);
    if (entity !== null) {
        if (entity.className === 'range-wrap') updateRange(entity, value);
        else if (entity.className === 'button') updateButton(entity, value);
        else console.log(entity.className);
    }
}

function getValue(reqId) {
    var xh = new XMLHttpRequest();
    xh.onreadystatechange = ()=>{if (xh.readyState === 4 && xh.status === 200) Object.entries(JSON.parse(xh.responseText)).forEach((entry) => parseEntry(entry))};
    xh.open('GET', `/sduino/data?${reqId}=0`, true);
    xh.send(null);
}

function updateValue(reqId, val) {
    var xh = new XMLHttpRequest();
    //XHR.addEventListener('error', () => { alert('Unable to send data!'); });
    xh.addEventListener('load', () => { getValue(reqId); });
    xh.open('POST', `/sduino/data?${reqId}=${val}`);
    xh.send(null);
}


// document.getElementById("defaultOpen").click();


// dataObj = {};
// propertyName = id;
// dataObj[propertyName] = val;
// sendData(dataObj);

// function sendData(data) {
//     const xh = new XMLHttpRequest();
//     const FD = new FormData();
//     for (let [name, value] of Object.entries(data)) { FD.append(name, value); }
//     //XHR.addEventListener('error', (event) => { alert('Unable to send data!'); });
//     xh.open('POST', '/sduino/data');
//     xh.send(FD);
// }

function initButtons() {
    var allButtons = document.querySelectorAll('.button');

    allButtons.forEach((butt) => {
        butt.addEventListener('click', () => {
            updateValue(butt.id, butt.value > 0 ? 0 : 1);
        });
    });
}

function initRanges() {
    var allRanges = document.querySelectorAll('.range-wrap');
    allRanges.forEach((wrap) => {
        var range = wrap.querySelector('.range');
        var bubble = wrap.querySelector('.bubble');
        var btns = wrap.querySelectorAll('.btnpm');

        range.addEventListener('input', () => {
            setBubble(range, bubble);
            updateValue(range.parentNode.id, range.value);
        });

        btns.forEach((btn) => {
            btn.addEventListener('click', () => {
                var newVal = Number(range.value) + Number(btn.innerHTML);
                range.value = newVal;
                setBubble(range, bubble);
                updateValue(range.parentNode.id, range.value);
            });
        });
    });
}

function switchTab(TabName) {
    var tablinks = document.getElementsByClassName('tablinks');
    var tablink = document.getElementById('LINK_' + TabName);
    var tabs = document.getElementsByClassName('tabcontent');
    var tab = document.getElementById('TAB_' + TabName);
    for (var i = 0; i < tabs.length; i++) tabs[i].style.display = 'none';
    for (var i = 0; i < tablinks.length; i++) tablinks[i].className = tablinks[i].className.replace(' active', '');
    tab.style.display = 'block';
    tablink.className += ' active';

    var allRanges = tab.querySelectorAll('.range-wrap');
    allRanges.forEach((wrap) => {
        var range = wrap.querySelector('.range');
        var bubble = wrap.querySelector('.bubble');
        setBubble(range, bubble);
    });
}

function initTabs() {
    var tablinks = document.getElementsByClassName('tablinks');
    for (var i = 0; i < tablinks.length; i++) {
        var tmp = tablinks[i];
        tmp.addEventListener('click', () => { switchTab(tmp.value); });
    }
}

function onBodyLoad() {
    initTabs();
    initButtons();
    initRanges();
    getValue('ALL');
    //getValue('CMDS')
    //setTimeout(loadCmds, 1000);
}

function testComms() {
    updateValue('0', '19.97');
}
