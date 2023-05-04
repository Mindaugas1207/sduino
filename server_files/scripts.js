
function loadValues(){
	var xh = new XMLHttpRequest();
	xh.onreadystatechange = function(){
		if (xh.readyState == 4 && xh.status == 200){
		var res = JSON.parse(xh.responseText);
		Object.entries(res).forEach((entry) => {
			const [id, value] = entry;
			const wrap = document.getElementById(id);
			if (wrap !== null) {
				const range = wrap.querySelector(".range");
				const bubble = wrap.querySelector(".bubble");
				if (range !== null || bubble !== null) {
					range.value = value;
					setBubble(range, bubble);
				}
			}
			else
			{
				const but = document.getElementById(id+":v");
				if (but !== null)
				{
					if (value > 0) {
						but.style.backgroundColor = "green";
					}
					if (value <= 0) {
						but.style.backgroundColor = "red";
					}	
				}
				else
				{
					const butb = document.getElementById(id+":"+"value");
					if (but !== null) {
						butb.style.backgroundColor = "green";
					}
				}
			}
		});
		}
	};
	xh.addEventListener('error', (event) => { alert('Unable to get data!'); });
	xh.open("GET", "/sduino/data?ALL=0", true);
	xh.send(null);
}

function loadCmds(){
	var xh = new XMLHttpRequest();
	xh.onreadystatechange = function(){
		if (xh.readyState == 4 && xh.status == 200){
		var res = JSON.parse(xh.responseText);
		Object.entries(res).forEach((entry) => {
			const [id, value] = entry;
			const wrap = document.getElementById(id);
			if (wrap !== null) {
				const range = wrap.querySelector(".range");
				const bubble = wrap.querySelector(".bubble");
				if (range !== null || bubble !== null) {
					range.value = value;
					setBubble(range, bubble);
				}
			}
			else
			{
				const but = document.getElementById(id+":v");
				if (but !== null)
				{
					if (value > 0) {
						but.style.backgroundColor = "green";
					}
					if (value <= 0) {
						but.style.backgroundColor = "red";
					}	
				}
				else
				{
					const butb = document.getElementById(id+":"+"value");
					if (but !== null) {
						butb.style.backgroundColor = "green";
					}
				}
			}
		});
		}
	};
	xh.addEventListener('error', (event) => { alert('Unable to get data!'); });
	xh.open("GET", "/sduino/data?CMDS=0", true);
	xh.send(null);
}

function getValue(id){
	var xh = new XMLHttpRequest();
	xh.onreadystatechange = function(){
		if (xh.readyState == 4 && xh.status == 200){
		var res = JSON.parse(xh.responseText);
		Object.entries(res).forEach((entry) => {
			const [id, value] = entry;
			const wrap = document.getElementById(id);
			if (wrap !== null) {
				const range = wrap.querySelector(".range");
				const bubble = wrap.querySelector(".bubble");
				if (range !== null || bubble !== null) {
					range.value = value;
					setBubble(range, bubble);
				}
			}
			else
			{
				const but = document.getElementById(id+":v");
				if (but !== null)
				{
					if (value > 0) {
						but.style.backgroundColor = "green";
					}
					if (value <= 0) {
						but.style.backgroundColor = "red";
					}	
				}
				else
				{
					const butb = document.getElementById(id+":"+"value");
					if (but !== null) {
						butb.style.backgroundColor = "green";
					}
				}
			}
		});
		}
	};
	xh.addEventListener('error', (event) => { alert('Unable to get data!'); });
	xh.open("GET", "/sduino/data?" + id + "=0", true);
	xh.send(null);
}

//document.getElementById("defaultOpen").click();
var thumbWidth = 20;

function setBubble(range, bubble){
	var newValue = Number(((range.value - range.min) * (range.clientWidth - thumbWidth) / (range.max - range.min)) - (bubble.clientWidth / 2) + (thumbWidth / 1.6));
	bubble.innerHTML = range.value;
	if (bubble.getAttribute('data-defval') === null) { bubble.setAttribute('data-defval', range.value); }
	var backValue = (range.value-range.min)/(range.max-range.min)*100;
	range.style.background = 'linear-gradient(to right, Teal 0%, Cyan ' + backValue + '%, LightGray ' + backValue + '%, white 100%)';
	bubble.style.left = newValue + 'px';
}

//dataObj = {};
//propertyName = id;
//dataObj[propertyName] = val;
//sendData(dataObj);

function sendData(data) {
	const XHR = new XMLHttpRequest();
	const FD = new FormData();
	for (const [name, value] of Object.entries(data)) { FD.append(name, value); }
	XHR.addEventListener('error', (event) => { alert('Unable to send data!'); });
	XHR.open('POST', '/sduino/data');
	XHR.send(FD);
}

function updateVar(id, val) {
	const XHR = new XMLHttpRequest();
	XHR.addEventListener('error', (event) => { alert('Unable to send data!'); });
	XHR.addEventListener('load', (event) => { getValue(id); });
	XHR.open("POST", "/sduino/data?" + id + "=" + val);
	XHR.send(null);
}

function onBodyLoad(){
	const allRanges = document.querySelectorAll(".range-wrap");
	allRanges.forEach(wrap => {
		const range = wrap.querySelector(".range");
		const bubble = wrap.querySelector(".bubble");
		
		range.addEventListener("input", () => {
			setBubble(range, bubble);
			updateVar(range.parentNode.id, range.value);
		});
		setBubble(range, bubble);
	});
	loadValues();
}

function switchTab(evt, TabName) {
	var i, tabcontent, tablinks;
	tabcontent = document.getElementsByClassName("tabcontent");
	for (i = 0; i < tabcontent.length; i++) {
		tabcontent[i].style.display = "none";
	}
	tablinks = document.getElementsByClassName("tablinks");
	for (i = 0; i < tablinks.length; i++) {
		tablinks[i].className = tablinks[i].className.replace(" active", "");
	}
	tab = document.getElementById(TabName);
	tab.style.display = "block";
	const allRanges = tab.querySelectorAll(".range-wrap");
	allRanges.forEach(wrap => {
		const range = wrap.querySelector(".range");
		const bubble = wrap.querySelector(".bubble");
		setBubble(range, bubble);
	});
	evt.currentTarget.className += " active";
}

function testComms() {
  updateVar("0", "19.97");
  getValue("0");
}

