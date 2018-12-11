/* This is an instantiation of a server that we designed:
-- to run on a local machine (laptop)
-- creates a web server that will communicate with the local USB port
-- serves a webpage to a client that will plot using a javscript chart library
-- server side
October 2018 -- Emily Lam
*/


// Modules
var SerialPort = require('serialport');
var Readline = require('@serialport/parser-readline')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var request = require('request');

/*
// Open serial port
var port = new SerialPort('/COM3', {baudRate: 115200});

// Read data from serial
var read_data;
var msg;
const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function (data) {
  read_data = data;
  console.log('Read:', read_data);
  msg = parseInt(read_data);  // Convert to int
  io.emit('message', msg);
});
*/

   // Test stuff --> no serial port
   
   
   setInterval( function() {
      const https = require('http');
       let data = '';

      https.get('http://192.168.1.127/gettemp', (resp) => {
        data = '';

        // A chunk of data has been recieved.
        resp.on('data', (chunk) => {
          data += chunk;
        });

        // The whole response has been received. Print out the result.
        resp.on('end', () => {
          //console.log(data);
          console.log('Read:', data);
          io.emit('message', data);
        });

      }).on("error", (err) => {
        console.log("Error: " + err.message);
      });

    https.get('http://192.168.1.129/gettemp', (resp) => {
        data = '';

        // A chunk of data has been recieved.
        resp.on('data', (chunk) => {
          data += chunk;
        });

        // The whole response has been received. Print out the result.
        resp.on('end', () => {
          //console.log(data);
          console.log('Read:', data);
          io.emit('message1', data);
        });

      }).on("error", (err) => {
        console.log("Error: " + err.message);
      });

    https.get('http://192.168.1.127/getsettemp', (resp) => {
        data = '';

        // A chunk of data has been recieved.
        resp.on('data', (chunk) => {
          data += chunk;
        });

        // The whole response has been received. Print out the result.
        resp.on('end', () => {
          //console.log(data);
          console.log('Read:', data);
          io.emit('message2', data);
        });

      }).on("error", (err) => {
        console.log("Error: " + err.message);
      });

    https.get('http://192.168.1.129/getsettemp', (resp) => {
        data = '';

        // A chunk of data has been recieved.
        resp.on('data', (chunk) => {
          data += chunk;
        });

        // The whole response has been received. Print out the result.
        resp.on('end', () => {
          //console.log(data);
          console.log('Read:', data);
          io.emit('message3', data);
        });

      }).on("error", (err) => {
        console.log("Error: " + err.message);
      });

     
   }, 5000);

   


// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// User socket connection
io.on('connection', function(socket){
  //console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
 
   socket.on('later',function (data){
    console.log(data);
    const options = {
    url: 'http://192.168.1.127/settemp',
    body: data
    };

    request.put(options);
  });

   socket.on('later1',function (data){
    console.log(data);
    const options = {
    url: 'http://192.168.1.129/settemp',
    body: data
    };

    request.put(options);
  });

  socket.on('power',function (data){
      const options = {
      url: 'http://192.168.1.127/power',
      body: data
      };

      request.put(options);
    });

  socket.on('power1',function (data){
      const options = {
      url: 'http://192.168.1.129/power',
      body: data
      };

      request.put(options);
    });

});
// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});

/*
var request = require('request');
request('192.168.1.123:80/ctrl', function (error, response, body) {
  console.log('error:', error); // Print the error if one occurred
  console.log('statusCode:', response && response.statusCode); // Print the response status code if a response was received
  console.log('body:', body); // Print the HTML for the Google homepage.
});
*/