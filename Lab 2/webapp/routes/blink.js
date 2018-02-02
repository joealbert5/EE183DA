var express = require('express');
var fs = require('fs');
var router = express.Router();

/* GET users listing. */
router.post('/', function(req, res, next) {
  //res.send('true');
  /*var title = req.body.title;
  var bodyy = req.body.bodyy;
  var userId = req.body.userId;
  console.log('test');
  console.log(req.body);
  res.send('title is: ' + title + ' and bodyy is: ' + bodyy + ' and userId is: ' + userId);
  */
  console.log('received POST from button');
  console.log(req.body);
  console.log(req.body.blk);
  var received = req.body.blk;
  if(received != "" || received != null){
  	fs.writeFile('public/db.txt', received, function (err, file) {
	  if (err) throw err;
	  console.log('Saved!');
	});
  }
  var text = fs.readFileSync("public/db.txt", "utf-8");
  console.log(text);
  res.send(text.toString());
});

router.get('/', function(req, res, next) {
  /*fs.readfile("public/db.txt", function(text){
	console.log(text);
  	res.send(text);
  });*/
  console.log('received GET from ESP');
  var text = fs.readFileSync("public/db.txt", "utf-8");
  console.log(text);
  res.send(text.toString());
});

module.exports = router;