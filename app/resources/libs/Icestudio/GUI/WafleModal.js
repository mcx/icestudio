class WafleModal {
  constructor() {}

  waitingSeconds(seconds, title, msg) {
    const milis = parseInt(Math.floor(seconds * 1000));
    console.log('WAIT==>', seconds, title, msg);
    let timerInterval;
    Swal.fire({
      title: title,
      html: msg,
      timer: milis,
      allowOutsideClick: false,
      allowEscapeKey: true,
      timerProgressBar: true,
      didOpen: () => {
        Swal.showLoading();
        const timer = Swal.getPopup().querySelector('b');
        timerInterval = setInterval(() => {
          timer.textContent = `${parseInt(Math.round(Swal.getTimerLeft() / 1000))}`;
        }, 100);
      },
      willClose: () => {
        clearInterval(timerInterval);
      },
    }).then((result) => {
      /* Read more about handling dismissals below */
      if (result.dismiss === Swal.DismissReason.timer) {
        console.log('Timmer end');
      }
    });
  }
}
