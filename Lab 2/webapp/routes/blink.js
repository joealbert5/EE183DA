//page that ESP parses to obtain data
var express = require('express');
var fs = require('fs');
var router = express.Router();

/* GET users listing. */
router.post('/', function(req, res, next) {
  console.log('received POST from button');
  console.log(req.body);
  console.log(req.body.blk);
  var received = req.body.blk;
  //save encoded data to database (text file on local machine)
  if(received != "" || received != null){
  	fs.writeFile('public/db.txt', received, function (err, file) {
	  if (err) throw err;
	  console.log('Saved!');
	});
  }
  //read from text file database, update ./blink.html with contents
  var text = fs.readFileSync("public/db.txt", "utf-8");
  console.log(text);
  res.send(text.toString());
});

//when ESP asks what the data is, it parses this page and downloads the data
router.get('/', function(req, res, next) {
  console.log('received GET from ESP');
  var text = fs.readFileSync("public/db.txt", "utf-8");
  console.log(text);
  res.send(text.toString());
});

module.exports = router;